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

#include "aruwsrc/robot/engineer/arm/wrist_fold_out_command.hpp"

namespace aruwsrc::engineer
{
WristFoldOutCommand::WristFoldOutCommand(WristSubsystem &wrist) : wrist(wrist)
{
    addSubsystemRequirement(&wrist);
}

void WristFoldOutCommand::initialize() {}

void WristFoldOutCommand::execute()
{
    switch (state)
    {
        case TOP:
            wrist.setSetpointPitch(1.5f);
            wrist.setSetpointYaw(0.0f);
            if (wrist.atSetpoint()) state = BOTTOM;
            break;
        case BOTTOM:
            wrist.setSetpointPitch(1.5f);
            wrist.setSetpointYaw(M_PI);
            if (wrist.atSetpoint()) state = OUT;
            break;
        case OUT:
            wrist.setSetpointPitch(0.0f);
            wrist.setSetpointYaw(M_PI);
            if (wrist.atSetpoint()) state = COMPLETED;
            break;
        case COMPLETED:
            break;
    }
}

void WristFoldOutCommand::end(bool) {}

bool WristFoldOutCommand::isFinished() const { return state == COMPLETED; }
}  // namespace aruwsrc::engineer
