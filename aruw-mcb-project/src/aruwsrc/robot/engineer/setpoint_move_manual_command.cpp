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

#include "setpoint_move_manual_command.hpp"

namespace aruwsrc::robot::engineer
{
SetpointMoveManualCommand::SetpointMoveManualCommand(
    LimitSwitchSetpointInterface& cubeLift,
    aruwsrc::control::engineer::EngineerControlOperatorInterface* operatorInterface,
    float moveSpeed)
    : cubeLift(cubeLift),
      operatorInterface(operatorInterface),
      moveSpeed(moveSpeed)
{
    addSubsystemRequirement(&cubeLift);
}

void SetpointMoveManualCommand::initialize()
{
    setpoint = cubeLift.getSetpoint();
    cubeLift.setPIDState(PIDState::POSITION_PID);
}

void SetpointMoveManualCommand::execute()
{
    if (!operatorInterface->isGantryControlMode()) return;

    setpoint += operatorInterface->getCubeLiftVelocity() * moveSpeed;
    cubeLift.setSetpoint(setpoint);
}

void SetpointMoveManualCommand::end(bool) { cubeLift.setDesiredOutput(0); }

bool SetpointMoveManualCommand::isFinished() const { return false; }
}  // namespace aruwsrc::robot::engineer
