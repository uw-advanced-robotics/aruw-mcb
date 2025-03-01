/*
 * Copyright (c) 2020-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "balstd_chassis_subsystem.hpp"

using namespace tap::algorithms;

namespace aruwsrc::control::balstd
{
BalstdChassisSubsystem::BalstdChassisSubsystem(
    tap::Drivers* drivers,
    tap::motor::DjiMotor* leftMotor,
    tap::motor::DjiMotor* leftMidMotor,
    tap::motor::DjiMotor* rightMidMotor,
    tap::motor::DjiMotor* rightMotor,
    tap::communication::sensors::current::CurrentSensorInterface* currentSensor)
    : HolonomicChassisSubsystem(drivers, currentSensor, nullptr),
      motors{leftMotor, leftMidMotor, rightMidMotor, rightMotor}
{
}

void BalstdChassisSubsystem::initialize()
{
    for (auto motor : motors)
    {
        motor->initialize();
    }
}

bool BalstdChassisSubsystem::allMotorsOnline() const
{
    bool online = true;
    for (auto motor : motors) online &= motor->isMotorOnline();
    return online;
}

void BalstdChassisSubsystem::setZeroRPM()
{
    for (auto motor : motors) motor->setDesiredOutput(0);
}

void BalstdChassisSubsystem::setDesiredOutput(float x, float y, float r) {}

void BalstdChassisSubsystem::refresh() {}

void BalstdChassisSubsystem::limitChassisPower()
{
    // use power limiting object to compute initial power limiting fraction
    currentSensor->update();
    float powerLimitFrac = chassisPowerLimiter.getPowerLimitRatio();
}

modm::Matrix<float, 3, 1> BalstdChassisSubsystem::getActualVelocityChassisRelative() const
{
    modm::Matrix<float, 8, 1> actualModuleVectors;
    for (unsigned int i = 0; i < 4; i++)
    {
        modm::Matrix<float, 2, 1> moduleVel = motors[i]->getActualModuleVelocity();
        actualModuleVectors[2 * i][0] = moduleVel[0][0];
        actualModuleVectors[2 * i + 1][0] = moduleVel[1][0];
    }
    return forwardMatrix * actualModuleVectors;
}

modm::Matrix<float, 3, 1> BalstdChassisSubsystem::getDesiredVelocityChassisRelative() const
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

}  // namespace aruwsrc::control::balstd
