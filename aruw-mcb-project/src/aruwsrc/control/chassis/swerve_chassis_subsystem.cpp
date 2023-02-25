/*
 * Copyright (c) 2022-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "swerve_chassis_subsystem.hpp"

using namespace tap::algorithms;

namespace aruwsrc
{
namespace chassis
{

SwerveChassisSubsystem::SwerveChassisSubsystem(
    tap::Drivers* drivers,
    SwerveModuleConfig config1,
    SwerveModuleConfig config2,
    tap::gpio::Analog::Pin currentPin)
    : HolonomicChassisSubsystem(drivers, currentPin),
    NUM_MODULES(2),
    modules {
#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
            testing::NiceMock<aruwsrc::mock::SwerveModuleMock>(drivers, config1), 
            testing::NiceMock<aruwsrc::mock::SwerveModuleMock>(drivers, config2), 
            testing::NiceMock<aruwsrc::mock::SwerveModuleMock>(drivers, DEFAULT_SWERVE_CONFIG), 
            testing::NiceMock<aruwsrc::mock::SwerveModuleMock>(drivers, DEFAULT_SWERVE_CONFIG)
#else
            SwerveModule(drivers, config1), 
            SwerveModule(drivers, config2), 
            SwerveModule(drivers, DEFAULT_SWERVE_CONFIG), 
            SwerveModule(drivers, DEFAULT_SWERVE_CONFIG)
#endif
            }
{
}

SwerveChassisSubsystem::SwerveChassisSubsystem(
    tap::Drivers* drivers,
    SwerveModuleConfig config1,
    SwerveModuleConfig config2,
    SwerveModuleConfig config3,
    SwerveModuleConfig config4,
    tap::gpio::Analog::Pin currentPin)
    : HolonomicChassisSubsystem(drivers, currentPin),
    NUM_MODULES(4),
    modules{
#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
            testing::NiceMock<aruwsrc::mock::SwerveModuleMock>(drivers, config1), 
            testing::NiceMock<aruwsrc::mock::SwerveModuleMock>(drivers, config2), 
            testing::NiceMock<aruwsrc::mock::SwerveModuleMock>(drivers, config3), 
            testing::NiceMock<aruwsrc::mock::SwerveModuleMock>(drivers, config4)
#else
            SwerveModule(drivers, config1), 
            SwerveModule(drivers, config2), 
            SwerveModule(drivers, config3), 
            SwerveModule(drivers, config4)
#endif
            }
{
}

// SwerveChassisSubsystem::SwerveChassisSubsystem(
//     tap::Drivers* drivers,
//     SwerveModule& module1,
//     SwerveModule& module2,
//     tap::gpio::Analog::Pin currentPin)
//     : HolonomicChassisSubsystem(drivers, currentPin),
//     modules{module1, 
//             module2, 
//             SwerveModule(drivers, DEFAULT_SWERVE_CONFIG), 
//             SwerveModule(drivers, DEFAULT_SWERVE_CONFIG)}
// {
// }

// SwerveChassisSubsystem::SwerveChassisSubsystem(
//     tap::Drivers* drivers,
//     SwerveModule& module1,
//     SwerveModule& module2,
//     SwerveModule& module3,
//     SwerveModule& module4,
//     tap::gpio::Analog::Pin currentPin)
//     : HolonomicChassisSubsystem(drivers, currentPin),
//     modules{module1, 
//             module2, 
//             module3, 
//             module4}
// {
// }


void SwerveChassisSubsystem::initialize()
{
    for(unsigned int i = 0; i < NUM_MODULES; i++)
    {
        modules[i].initialize();
    }
}


bool SwerveChassisSubsystem::allMotorsOnline() const
{
    
    bool online = true;
    for (unsigned int i = 0; i < NUM_MODULES; i++)
        online &= modules[i].allMotorsOnline();
    return online;
}


void SwerveChassisSubsystem::setZeroRPM()
{
    for (unsigned int i = 0; i < NUM_MODULES; i++)
        modules[i].setZeroRPM();
}


void SwerveChassisSubsystem::setDesiredOutput(float x, float y, float r)
{
    //convert inputs from motor rpm to m/s
    x = modules[0].rpmToMps(x);
    y = modules[0].rpmToMps(y);
    r = modules[0].rpmToMps(r) / WIDTH_BETWEEN_WHEELS_X * 2;
    //^simplified tank drive rotation calculation that doesnt take width_y into account
    swerveDriveCalculate(
        x,
        y,
        r,
        getMaxWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            drivers->refSerial.getRobotData().chassis.powerConsumptionLimit));
}


void SwerveChassisSubsystem::swerveDriveCalculate(float x, float y, float r, float maxWheelRPM)
{
    desiredRotation = modules[0].mpsToRpm(r) * WIDTH_BETWEEN_WHEELS_X / 2;
    float maxInitialSpeed = 0;
    for(unsigned int i = 0; i < NUM_MODULES; i++)
    {
        desiredModuleSpeeds[i][0] = modules[i].calculate(x, y, r);
        maxInitialSpeed = std::max(maxInitialSpeed, desiredModuleSpeeds[i][0]);
    }

    float scaleCoeff = std::min(maxWheelRPM / maxInitialSpeed, 1.0f);

    for(unsigned int i = 0; i < NUM_MODULES; i++)
    {
        modules[i].scaleAndSetDesiredState(scaleCoeff);
    }
}


void SwerveChassisSubsystem::refresh()
{
    for(unsigned int i = 0; i < NUM_MODULES; i++)
    {
        modules[i].refresh();
    }
}


void SwerveChassisSubsystem::limitChassisPower()
{
    // use power limiting object to compute initial power limiting fraction
    currentSensor.update();
    float powerLimitFrac = chassisPowerLimiter.getPowerLimitRatio();

    // short circuit if power limiting doesn't need to be applied
    if (compareFloatClose(1.0f, powerLimitFrac, 1E-3))
    {
        return;
    }

    // total velocity error for all wheels
    float totalError = 0.0f;
    for (unsigned int i = 0; i < NUM_MODULES; i++)
    {
        totalError += abs(modules[i].calculateTotalModuleError());
    }

    bool totalErrorZero = compareFloatClose(0.0f, totalError, 1E-3);

    // compute modified power limiting fraction based on velocity PID error
    // motors with greater error should be allocated a larger fraction of the powerLimitFrac
    for (unsigned int i = 0; i < NUM_MODULES; i++)
    {
        // Compared to the other wheels, fraction of how much velocity PID error there is for a
        // single motor. Some value between [0, 1]. The sum of all computed velocityErrorFrac
        // values for all motors is 1.
        float velocityErrorFrac = totalErrorZero
                                      ? (1.0f / NUM_MODULES)
                                      : (abs(modules[i].calculateTotalModuleError()) / totalError);
        // Instead of just multiplying the desired output by powerLimitFrac, scale powerLimitFrac
        // based on the current velocity error. In this way, if the velocity error is large, the
        // motor requires more current to be directed to it than other motors. Without this
        // compensation, a total of NUM_MOTORS * powerLimitFrac fractional limiting is divided
        // evenly among NUM_MOTORS motors. Instead, divide this limiting based on the
        // velocityErrorFrac for each motor.
        float modifiedPowerLimitFrac =
            limitVal(NUM_MODULES * powerLimitFrac * velocityErrorFrac, 0.0f, 1.0f);
        //modules[i].limitPower(modifiedPowerLimitFrac);
    }
}


modm::Matrix<float, 3, 1> SwerveChassisSubsystem::getActualVelocityChassisRelative() const
{
    modm::Matrix<float, 3, 1> randomOutput;
    randomOutput[0][0] = 0;
    randomOutput[1][0] = 0;
    randomOutput[2][0] = 0;
    return randomOutput;
}

modm::Matrix<float, 3, 1> SwerveChassisSubsystem::getDesiredVelocityChassisRelative() const
{
    return getActualVelocityChassisRelative();
}

}

}
