/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "aruwsrc/robot/engineer/setpoint_move_manual_command.hpp"

namespace aruwsrc::engineer
{
SetpointMoveManualCommand::SetpointMoveManualCommand(
    aruwsrc::control::joint::JointSubsystem& subsystem,
    aruwsrc::engineer::EngineerControlOperatorInterface* operatorInterface,
    float moveSpeed,
    SetpointType setpointType)
    : subsystem(subsystem),
      operatorInterface(operatorInterface),
      moveSpeed(moveSpeed),
      setpointType(setpointType)
{
    addSubsystemRequirement(&subsystem);
}

void SetpointMoveManualCommand::initialize() {}

void SetpointMoveManualCommand::execute()
{
    if (!operatorInterface->isGantryWristControlMode()) return;

    float setpoint = subsystem.getSetpoint();
    switch (setpointType)
    {
        case SetpointType::CUBE_STORAGE:
            setpoint += operatorInterface->getCubeStorageVelocity() * moveSpeed;
            break;
        case SetpointType::EXTENSION:
            setpoint += operatorInterface->getGantryExtensionVelocity() * moveSpeed;
            if (operatorInterface->getGantryKeyOut())
            {
                setpoint += moveSpeed;
            }
            else if (operatorInterface->getGantryKeyIn())
            {
                setpoint -= moveSpeed;
            }
            break;
        default:
            break;  // Invalid setpoint type
    }
    subsystem.setSetpoint(setpoint);
}

void SetpointMoveManualCommand::end(bool) {}

bool SetpointMoveManualCommand::isFinished() const { return false; }
}  // namespace aruwsrc::engineer
