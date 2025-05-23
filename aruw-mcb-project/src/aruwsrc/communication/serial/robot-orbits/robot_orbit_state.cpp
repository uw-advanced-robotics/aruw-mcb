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

#include "robot_orbit_state.hpp"

namespace aruwsrc::communication::serial
{
int RobotOrbitStateProvider::findRobotIndex(RefSerialData::RobotId robotID) const
{
    for (uint8_t i = 0; i < MAX_TRACKED_ROBOTS; i++)
    {
        if (std::get<0>(robotData[i]) && std::get<1>(robotData[i]) == robotID)
        {
            return i;
        }
    }
    return -1;
}

void RobotOrbitStateProvider::updateRobotState(
    RefSerialData::RobotId robotID,
    const RobotState& state)
{
    int index = findRobotIndex(robotID);
    if (index != -1)
    {
        std::get<2>(robotData[index]) = state;
    }
    else
    {
        for (uint8_t i = 0; i < MAX_TRACKED_ROBOTS; i++)
        {
            if (!std::get<0>(robotData[i]))
            {
                std::get<0>(robotData[i]) = true;
                std::get<1>(robotData[i]) = robotID;
                std::get<2>(robotData[i]) = state;
                return;
            }
        }
    }
}

bool RobotOrbitStateProvider::getRobotState(RefSerialData::RobotId robotID, RobotState& outState)
    const
{
    int index = findRobotIndex(robotID);
    if (index != -1)
    {
        outState = std::get<2>(robotData[index]);
        return true;
    }
    return false;
}

uint8_t RobotOrbitStateProvider::getNumKnownVisionStates(
    RobotState states[MAX_TRACKED_ROBOTS]) const
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < MAX_TRACKED_ROBOTS; i++)
    {
        if (std::get<0>(robotData[i]))
        {
            states[count++] = std::get<2>(robotData[i]);
        }
    }
    return count;
}

RobotState RobotOrbitStateProvider::getRobotState(RefSerialData::RobotId robotID) const
{
    RobotState state{};
    getRobotState(robotID, state);
    return state;
}

}  // namespace aruwsrc::communication::serial
