/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "turret_user_control_command.hpp"

#include "../turret_subsystem.hpp"
#include "aruwsrc/drivers.hpp"

namespace aruwsrc::control::turret
{
TurretUserControlCommand::TurretUserControlCommand(
    aruwsrc::Drivers *drivers,
    TurretSubsystem *turretSubsystem,
    TurretYawControllerInterface *yawController,
    TurretPitchControllerInterface *pitchController)
    : drivers(drivers),
      turretSubsystem(turretSubsystem),
      yawController(yawController),
      pitchController(pitchController)
{
    addSubsystemRequirement(turretSubsystem);
}

bool TurretUserControlCommand::isReady() { return !isFinished(); }

void TurretUserControlCommand::initialize()
{
    yawController->initialize();
    pitchController->initialize();
}

void TurretUserControlCommand::execute()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    const float pitchSetpoint =
        pitchController->getSetpoint() +
        USER_PITCH_INPUT_SCALAR * drivers->controlOperatorInterface.getTurretPitchInput();
    pitchController->runController(dt, pitchSetpoint);

    const float yawSetpoint =
        yawController->getSetpoint() +
        USER_YAW_INPUT_SCALAR * drivers->controlOperatorInterface.getTurretYawInput();
    yawController->runController(dt, yawSetpoint);
}

bool TurretUserControlCommand::isFinished() const
{
    return yawController->isFinished() || pitchController->isFinished();
}

void TurretUserControlCommand::end(bool)
{
    turretSubsystem->setPitchMotorOutput(0);
    turretSubsystem->setYawMotorOutput(0);
}

}  // namespace aruwsrc::control::turret
