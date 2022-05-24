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

#include "stepper_motor_turret_control_command.hpp"

#include "../turret_subsystem.hpp"
#include "aruwsrc/drivers.hpp"

namespace aruwsrc::control::turret::user
{
StepperMotorTurretControlCommand::StepperMotorTurretControlCommand(
    aruwsrc::Drivers *drivers,
    TurretSubsystem& turretSubsystem)
    : drivers(drivers),
      turretPitchMotor(turretSubsystem.pitchMotor),
      turretYawMotor(turretSubsystem.yawMotor)
{
    addSubsystemRequirement(&turretSubsystem);
}

bool StepperMotorTurretControlCommand::isReady() { return !isFinished(); }

void StepperMotorTurretControlCommand::initialize() {}

void StepperMotorTurretControlCommand::execute()
{
    turretPitchMotor.setMotorOutput(
        drivers->controlOperatorInterface.getTurretPitchInput(0));
    turretYawMotor.setMotorOutput(
        drivers->controlOperatorInterface.getTurretYawInput(0));
}

bool StepperMotorTurretControlCommand::isFinished() const { return false; }

void StepperMotorTurretControlCommand::end(bool)
{
    turretPitchMotor.setMotorOutput(0);
    turretYawMotor.setMotorOutput(0);
}

}  // namespace aruwsrc::control::turret::user
