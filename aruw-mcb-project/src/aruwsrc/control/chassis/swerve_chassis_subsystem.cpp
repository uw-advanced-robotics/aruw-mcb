/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

/*
 * Copyright (c) 2019 Sanger_X
 */

#include "swerve_chassis_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/serial/remote.hpp"

#include "aruwsrc/communication/sensors/current/acs712_current_sensor_config.hpp"
#include "aruwsrc/drivers.hpp"

#include "swerve_module.hpp"

using namespace tap::algorithms;

namespace aruwsrc
{
namespace chassis
{

SwerveChassisSubsystem::SwerveChassisSubsystem(
    aruwsrc::Drivers* drivers,
    tap::motor::MotorId leftFrontAzimuthMotorId,
    tap::motor::MotorId leftFrontDriveMotorId,
    tap::motor::MotorId leftBackAzimuthMotorId,
    tap::motor::MotorId leftBackDriveMotorId,
    tap::motor::MotorId rightFrontAzimuthMotorId,
    tap::motor::MotorId rightFrontDriveMotorId,
    tap::motor::MotorId rightBackAzimuthMotorId,
    tap::motor::MotorId rightBackDriveMotorId,
    chassis::SwerveModuleConfig config,
    tap::gpio::Analog::Pin currentPin)
    : HolonomicChassisSubsystem(drivers, currentPin),
    modules{
        SwerveModule(drivers, leftFrontDriveMotorId, leftFrontAzimuthMotorId, config, 
            -WIDTH_BETWEEN_WHEELS_X/2 - GIMBAL_X_OFFSET, WIDTH_BETWEEN_WHEELS_Y/2 - GIMBAL_Y_OFFSET),
        SwerveModule(drivers, leftBackDriveMotorId, leftBackAzimuthMotorId, config, 
            -WIDTH_BETWEEN_WHEELS_X/2 - GIMBAL_X_OFFSET, -WIDTH_BETWEEN_WHEELS_Y/2 - GIMBAL_Y_OFFSET),
        SwerveModule(drivers, rightFrontDriveMotorId, rightFrontAzimuthMotorId, config, 
            WIDTH_BETWEEN_WHEELS_X/2 - GIMBAL_X_OFFSET, WIDTH_BETWEEN_WHEELS_Y/2 - GIMBAL_Y_OFFSET),
        SwerveModule(drivers, rightBackDriveMotorId, rightBackAzimuthMotorId, config, 
            WIDTH_BETWEEN_WHEELS_X/2 - GIMBAL_X_OFFSET, -WIDTH_BETWEEN_WHEELS_Y/2 - GIMBAL_Y_OFFSET)
    }
{
}

void SwerveChassisSubsystem::initialize()
{
    for(int i = 0; i<4; i++)
    {
        modules[i].intialize();
    }
}

void SwerveChassisSubsystem::setDesiredOutput(float x, float y, float r)
{
    swerveDriveCalculate(
        x,
        y,
        r,
        getMaxWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            drivers->refSerial.getRobotData().chassis.powerConsumptionLimit));
}


void SwerveChassisSubsystem::swerveDriveCalculate(float x, float y, float r, float maxWheelSpeed)
{
    float maxInitialSpeed = 0;
    for(int i = 0; i<4; i++)
    {
        desiredModuleStates[i][0] = modules[i].calculate(x, y, r);
        if(desiredModuleStates[i][0] > maxInitialSpeed)
        {
            maxInitialSpeed = desiredModuleStates[i][0];
        }
    }

    float scaleCoeff = 1;
    if(maxInitialSpeed > maxWheelSpeed) scaleCoeff = maxWheelSpeed / maxInitialSpeed;

    for(int i = 0; i<4; i++)
    {
        modules[i].scaleAndSet(scaleCoeff);
    }
}

void SwerveChassisSubsystem::refresh()
{
    for(int i = 0; i<4; i++)
    {
        modules[i].refresh();
    }
}

void SwerveChassisSubsystem::limitChassisPower()
{
    int NUM_MOTORS = 4;

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
    for (int i = 0; i < 4; i++)
    {
        totalError += abs(modules[i].calculateTotalModuleError());

    }

    bool totalErrorZero = compareFloatClose(0.0f, totalError, 1E-3);

    // compute modified power limiting fraction based on velocity PID error
    // motors with greater error should be allocated a larger fraction of the powerLimitFrac
    for (int i = 0; i < 4; i++)
    {
        // Compared to the other wheels, fraction of how much velocity PID error there is for a
        // single motor. Some value between [0, 1]. The sum of all computed velocityErrorFrac
        // values for all motors is 1.
        float velocityErrorFrac = totalErrorZero
                                      ? (1.0f / 4)
                                      : (abs(modules[i].calculateTotalModuleError()) / totalError);
        // Instead of just multiplying the desired output by powerLimitFrac, scale powerLimitFrac
        // based on the current velocity error. In this way, if the velocity error is large, the
        // motor requires more current to be directed to it than other motors. Without this
        // compensation, a total of NUM_MOTORS * powerLimitFrac fractional limiting is divided
        // evenly among NUM_MOTORS motors. Instead, divide this limiting based on the
        // velocityErrorFrac for each motor.
        float modifiedPowerLimitFrac =
            limitVal(NUM_MOTORS * powerLimitFrac * velocityErrorFrac, 0.0f, 1.0f);
        //motors[i]->setDesiredOutput(motors[i]->getOutputDesired() * modifiedPowerLimitFrac);
        modules[i].limitPower(modifiedPowerLimitFrac);
    }
}


modm::Matrix<float, 3, 1> SwerveChassisSubsystem::getActualVelocityChassisRelative() const
{
    // modm::Matrix<float, MODM_ARRAY_SIZE(modules)*2, 1> wheelVelocity;
    // for(int i = 0; i<4; i++)
    // {
    //     float ang = modules[i].getAngle();
    //     float mag = modules[i].getDriveVelocity();
    //     wheelVelocity[2*i][0] = mag * cos(ang);
    //     wheelVelocity[2*i + 1][0] = mag * sin(ang);
    // }

    // wheelVelocity[LF][0] = leftFrontMotor.getShaftRPM();
    // wheelVelocity[RF][0] = rightFrontMotor.getShaftRPM();
    // wheelVelocity[LB][0] = leftBackMotor.getShaftRPM();
    // wheelVelocity[RB][0] = rightBackMotor.getShaftRPM();
    // return wheelVelToChassisVelMat * convertRawRPM(wheelVelocity);

    modm::Matrix<float, 3, 1> randomOutput;
    randomOutput[0][0] = 0;
    randomOutput[1][0] = 0;
    randomOutput[2][0] = 0;
    return randomOutput;
}




}

}