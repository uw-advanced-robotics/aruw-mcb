/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "dart_setpoint_command.hpp"

#include "aruwsrc/control/joint/homing/trigger_homed_joint_subsystem.hpp"

#include "dart_constants.hpp"
#include "dart_servo.hpp"

namespace aruwsrc::dart
{
DartSetpointCommand::DartSetpointCommand(
    aruwsrc::control::joint::homing::TriggerHomedJointSubsystem& pullMotorSubsystem,
    float setpoint)
    : pullMotorSubsystem(pullMotorSubsystem),
      setpoint(setpoint)
{
    addSubsystemRequirement(&pullMotorSubsystem);
}

void DartSetpointCommand::initialize() { pullMotorSubsystem.setSetpoint(setpoint); }

bool DartSetpointCommand::isFinished() const { return pullMotorSubsystem.atSetpoint(); }

}  // namespace aruwsrc::dart