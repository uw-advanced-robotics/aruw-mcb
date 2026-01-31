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

#include "aruwsrc/robot/engineer/wrist/wrist_move_position_command.hpp"
namespace aruwsrc::engineer::wrist
{
WristMovePositionCommand::WristMovePositionCommand(
    WristSubsystem &wrist,
    float pitchSetpoint,
    float yawSetpoint)
    : wrist(wrist),
      pitchSetpoint(pitchSetpoint),
      yawSetpoint(yawSetpoint)
{
    addSubsystemRequirement(&wrist);
}

void WristMovePositionCommand::initialize()
{
    wrist.setSetpointPitch(pitchSetpoint);
    wrist.setSetpointYaw(yawSetpoint);
}

void WristMovePositionCommand::execute() {}

void WristMovePositionCommand::end(bool) {}

bool WristMovePositionCommand::isFinished() const { return wrist.atSetpoint(); }
}  // namespace aruwsrc::engineer::wrist
