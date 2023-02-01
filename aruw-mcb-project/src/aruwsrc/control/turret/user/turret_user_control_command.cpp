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



#include "../turret_subsystem.hpp"
#include "aruwsrc/drivers.hpp"
#include "turret_user_control_command.hpp"
namespace aruwsrc::control::turret::user
{
TurretUserControlCommand::TurretUserControlCommand(
    float (*getTurretPitchInput) (uint8_t),
    float (*getTurretYawInput) (uint8_t),
    TurretSubsystem *turretSubsystem,
    algorithms::TurretYawControllerInterface *yawController,
    algorithms::TurretPitchControllerInterface *pitchController,
    float userYawInputScalar,
    float userPitchInputScalar,
    uint8_t turretID)
    : getTurretPitchInput(),
      getTurretYawInput(),
      turretSubsystem(turretSubsystem),
      yawController(yawController),
      pitchController(pitchController),
      userYawInputScalar(userYawInputScalar),
      userPitchInputScalar(userPitchInputScalar),
      turretID(turretID)
{
    addSubsystemRequirement(turretSubsystem);
}

bool TurretUserControlCommand::isReady() { return !isFinished(); }

void TurretUserControlCommand::initialize()
{
    yawController->initialize();
    pitchController->initialize();
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

void TurretUserControlCommand::execute()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    const float pitchSetpoint =
        pitchController->getSetpoint() +
        userPitchInputScalar * getTurretPitchInput(turretID);
    pitchController->runController(dt, pitchSetpoint);

    const float yawSetpoint =
        yawController->getSetpoint() +
        userYawInputScalar * getTurretYawInput(turretID);
    yawController->runController(dt, yawSetpoint);
}

bool TurretUserControlCommand::isFinished() const
{
    return !pitchController->isOnline() && !yawController->isOnline();
}

void TurretUserControlCommand::end(bool)
{
    turretSubsystem->yawMotor.setMotorOutput(0);
    turretSubsystem->pitchMotor.setMotorOutput(0);
}

}  // namespace aruwsrc::control::turret::user
