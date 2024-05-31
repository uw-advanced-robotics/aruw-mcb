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
    tap::communication::sensors::current::CurrentSensorInterface* currentSensor,
    Module* moduleLeftFront,
    Module* moduleRightFront,
    Module* moduleLeftBack,
    Module* moduleRightBack,
    const float forwardMatrixArray[24],
    can::capbank::CapacitorBank* capacitorBank)
    : HolonomicChassisSubsystem(drivers, currentSensor, capacitorBank),
      modules{moduleLeftFront, moduleRightFront, moduleLeftBack, moduleRightBack},
      forwardMatrix(forwardMatrixArray)
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
    x = modules[LF]->wheel.rpmToMps(x);           // convert input from motor rpm to m/s
    y = modules[LF]->wheel.rpmToMps(y);           // convert input from motor rpm to m/s
    r = modules[LF]->wheel.rpmToMps(r) / 0.205f;  // convert input from motor rpm to rad/s
    // TODO: REPLACE WITH CONSTANT FROM CONSTANTS FILE
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
    desiredRotation = modules[LF]->wheel.mpsToRpm(r) * 0.205f;
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
    currentSensor->update();
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
    modm::Matrix<float, 8, 1> actualModuleVectors;
    for (unsigned int i = 0; i < NUM_MODULES; i++)
    {
        modm::Matrix<float, 2, 1> moduleVel = modules[i]->getActualModuleVelocity();
        actualModuleVectors[2 * i][0] = moduleVel[0][0];
        actualModuleVectors[2 * i + 1][0] = moduleVel[1][0];
    }
    return forwardMatrix * actualModuleVectors;
}

modm::Matrix<float, 3, 1> SwerveChassisSubsystem::getDesiredVelocityChassisRelative() const
{
    modm::Matrix<float, 8, 1> desiredModuleVectors;
    for (unsigned int i = 0; i < NUM_MODULES; i++)
    {
        modm::Matrix<float, 2, 1> moduleVel = modules[i]->getDesiredModuleVelocity();
        desiredModuleVectors[2 * i][0] = moduleVel[0][0];
        desiredModuleVectors[2 * i + 1][0] = moduleVel[1][0];
    }
    return forwardMatrix * desiredModuleVectors;
}

}  // namespace chassis

}  // namespace aruwsrc
