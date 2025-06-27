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

#include "wrist_move_position_command.hpp"
namespace aruwsrc::engineer::wrist
{
WristMovePositionCommand::WristMovePositionCommand(
    WristSubsystem &wrist,
    float pitchSetpoint,
    float yawSetpoint)
    : wrist(wrist),
      pitchSetpoint(pitchSetpoint),
      yawSetpoint(yawSetpoint),
      currPitchSetpoint(0, 0, M_TWOPI),
      currYawSetpoint(0, 0, M_TWOPI)
{
    addSubsystemRequirement(&wrist);
}

void WristMovePositionCommand::initialize()
{
    currPitchSetpoint = wrist.getPitchWrapped();
    currYawSetpoint = wrist.getYawWrapped();
}

int Counter;
float erPitch, erYaw;

void WristMovePositionCommand::execute()
{
    Counter++;

    float errorPitch = currPitchSetpoint.minDifference(pitchSetpoint);
    erPitch = errorPitch;
    if (fabs(errorPitch) > WRIST_MOVE_POSITION_COMMAND_RAMP_EPSILON)
        currPitchSetpoint +=
            WRIST_MOVE_POSITION_COMMAND_RAMP_RATE * tap::algorithms::getSign(errorPitch);
    else
        currYawSetpoint.setWrappedValue(yawSetpoint);

    float errorYaw = currYawSetpoint.minDifference(yawSetpoint);
    erYaw = errorYaw;
    if (fabs(errorYaw) > WRIST_MOVE_POSITION_COMMAND_RAMP_EPSILON)
        currYawSetpoint +=
            WRIST_MOVE_POSITION_COMMAND_RAMP_RATE * tap::algorithms::getSign(errorYaw);
    else
        currYawSetpoint.setWrappedValue(yawSetpoint);

    wrist.setSetpointPitch(currPitchSetpoint.getWrappedValue());
    wrist.setSetpointYaw(currYawSetpoint.getWrappedValue());
}

bool EndState;

void WristMovePositionCommand::end(bool a) { EndState = a; }

int commandsCompleted;

bool WristMovePositionCommand::isFinished() const
{
    bool complete = wrist.atSetpoint() &&
                    (fabs(currPitchSetpoint.minDifference(pitchSetpoint)) <
                     WRIST_MOVE_POSITION_COMMAND_RAMP_EPSILON) &&
                    (fabs(currYawSetpoint.minDifference(yawSetpoint)) <
                     WRIST_MOVE_POSITION_COMMAND_RAMP_EPSILON);
    if (complete) commandsCompleted++;
    return complete;
}
}  // namespace aruwsrc::engineer::wrist
