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
#include "dart_manual_pullback_setpoint_command.hpp"

namespace aruwsrc::dart

{
DartManualPullbackSetpointCommand::DartManualPullbackSetpointCommand(
    aruwsrc::control::joint::homing::TriggerHomedJointSubsystem& dartSystem,
    float moveSpeed,
    aruwsrc::control::dart::DartControlOperatorInterface* controlOperatorInterface)
    : dartSystem(dartSystem),
      moveSpeed(moveSpeed),
      controlOperatorInterface(controlOperatorInterface)
{
    addSubsystemRequirement(&dartSystem);
}

void DartManualPullbackSetpointCommand::initialize() {}

void DartManualPullbackSetpointCommand::execute()
{
    float setpoint = dartSystem.getSetpoint();

    position = dartSystem.getPosition();
    setpoint += controlOperatorInterface->getPullbackVelocity() * moveSpeed;

    dartSystem.setSetpoint(setpoint);
}

bool DartManualPullbackSetpointCommand::isFinished() const { return false; }

}  // namespace aruwsrc::dart