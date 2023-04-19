/*
 * Copyright (c) 2022-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "motor_homing_command.hpp"

#include "tap/drivers.hpp"

namespace aruwsrc::control
{
void MotorHomingCommand::initialize() { homingState = HomingState::MOVING_TOWARD_LOWER_BOUND; }

void MotorHomingCommand::execute()
{
    switch (homingState)
    {
        case (HomingState::INITIATE_MOVE_TOWARD_LOWER_BOUND):
        {
            subsystem.moveTowardLowerBound();
            homingState = HomingState::MOVING_TOWARD_LOWER_BOUND;
            break;
        }
        case (HomingState::MOVING_TOWARD_LOWER_BOUND):
        {
            if (subsystem.isStalled())
            {
                subsystem.setLowerBound();
                subsystem.stop();
                homingState = HomingState::MOVING_TOWARD_UPPER_BOUND;
            }
            break;
        }
        case (HomingState::INITIATE_MOVE_TOWARD_UPPER_BOUND):
        {
            subsystem.moveTowardUpperBound();
            homingState = HomingState::MOVING_TOWARD_UPPER_BOUND;
            break;
        }
        case (HomingState::MOVING_TOWARD_UPPER_BOUND):
        {
            if (subsystem.isStalled())
            {
                subsystem.setUpperBound();
                subsystem.stop();
                homingState = HomingState::HOMING_COMPLETE;
            }
            break;
        }
        case (HomingState::HOMING_COMPLETE):
        {
            break;
        }
    }
}

void MotorHomingCommand::end(bool) { subsystem.stop(); }

bool MotorHomingCommand::isFinished() const
{
    return (homingState == HomingState::HOMING_COMPLETE);
}

}  // namespace aruwsrc::control
