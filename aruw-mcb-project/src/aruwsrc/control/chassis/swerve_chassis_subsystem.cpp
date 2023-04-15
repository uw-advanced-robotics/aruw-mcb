/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
    SwerveModule* moduleFront,
    SwerveModule* moduleLeft,
    SwerveModule* moduleBack,
    SwerveModule* moduleRight,
    tap::gpio::Analog::Pin currentPin)
    : HolonomicChassisSubsystem(drivers, currentPin),
      NUM_MODULES(4),
      modules{
          moduleFront,
          moduleLeft,
          moduleBack,
          moduleRight}
{
}

void SwerveChassisSubsystem::initialize()
{
    for (unsigned int i = 0; i < NUM_MODULES; i++)
    {
        modules[i]->initialize();
    }
}

bool SwerveChassisSubsystem::allMotorsOnline() const
{
    bool online = true;
    for (unsigned int i = 0; i < NUM_MODULES; i++) online &= modules[i]->allMotorsOnline();
    return online;
}

void SwerveChassisSubsystem::setZeroRPM()
{
    for (unsigned int i = 0; i < NUM_MODULES; i++) modules[i]->setZeroRPM();
}

Module* SwerveChassisSubsystem::getModule(unsigned int i)
{
    if (i >= NUM_MODULES) return nullptr;
    return modules[i];
}

void SwerveChassisSubsystem::setDesiredOutput(float x, float y, float r)
{
    x = modules[F]->wheel.rpmToMps(x);  // convert input from motor rpm to m/s
    y = modules[F]->wheel.rpmToMps(y);  // convert input from motor rpm to m/s
    r = modules[F]->wheel.rpmToMps(r) / 0.205f;  // convert input from motor rpm to rad/s
    //TODO: REPLACE WITH CONSTANT FROM CONSTANTS FILE
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
    desiredRotation = modules[LF]->wheel.mpsToRpm(r) * WIDTH_BETWEEN_WHEELS_X / 2;
    float maxInitialSpeed = 0;
    for (unsigned int i = 0; i < NUM_MODULES; i++)
    {
        desiredModuleSpeeds[i][0] = modules[i]->calculate(x, y, r);
        maxInitialSpeed = std::max(maxInitialSpeed, desiredModuleSpeeds[i][0]);
    }

    float scaleCoeff = std::min(maxWheelRPM / maxInitialSpeed, 1.0f);

    for (unsigned int i = 0; i < NUM_MODULES; i++)
    {
        modules[i]->scaleAndSetDesiredState(scaleCoeff);
    }
}

void SwerveChassisSubsystem::refresh()
{
    for (unsigned int i = 0; i < NUM_MODULES; i++)
    {
        modules[i]->refresh();
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

    for (unsigned int i = 0; i < NUM_MODULES; i++)
    {
        modules[i]->limitPower(powerLimitFrac);
    }
}

modm::Matrix<float, 3, 1> SwerveChassisSubsystem::getActualVelocityChassisRelative() const
{
    // TODO: calculate forward matrix and store in constants file
    modm::Matrix<float, 3, 1> randomOutput;
    randomOutput[0][0] = 0;
    randomOutput[1][0] = 0;
    randomOutput[2][0] = 0;
    return randomOutput;
}

modm::Matrix<float, 3, 1> SwerveChassisSubsystem::getDesiredVelocityChassisRelative() const
{
    return getActualVelocityChassisRelative();
    // TODO: oh god bruh think about it later
}

}  // namespace chassis

}  // namespace aruwsrc
