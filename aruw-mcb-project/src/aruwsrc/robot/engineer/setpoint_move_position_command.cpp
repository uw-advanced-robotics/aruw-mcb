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

#include "setpoint_move_position_command.hpp"

namespace aruwsrc::engineer
{
SetpointMovePositionCommand::SetpointMovePositionCommand(
    LimitSwitchSetpointInterface &subsystem,
    float setpoint)
    : subsystem(subsystem),
      setpoint(setpoint)
{
    addSubsystemRequirement(&subsystem);
}

void SetpointMovePositionCommand::initialize() { subsystem.setSetpoint(setpoint); }

void SetpointMovePositionCommand::execute() {}

void SetpointMovePositionCommand::end(bool) {}

bool SetpointMovePositionCommand::isFinished() const { return subsystem.atSetpoint(); }
}  // namespace aruwsrc::engineer
