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
    float theta1setpoint,
    float theta2setpoint,
    float theta3setpoint)

    : wrist(wrist),
      theta1setpoint(theta1setpoint),
      theta2setpoint(theta2setpoint),
      theta3setpoint(theta3setpoint)
{
    addSubsystemRequirement(&wrist);
}

void WristMovePositionCommand::initialize()
{
    wrist.setSetpointTheta2(theta2setpoint);  // theta2 is pitch
    wrist.setSetpointTheta1(theta1setpoint);  // theta1 is yaw
}

void WristMovePositionCommand::execute() {}

void WristMovePositionCommand::end(bool) {}

bool WristMovePositionCommand::isFinished() const { return wrist.atSetpoint(); }
}  // namespace aruwsrc::engineer::wrist
