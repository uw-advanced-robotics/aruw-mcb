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
RobotIndex RobotOrbitStateProvider::getRobotIndexFromId(RefSerialData::RobotId robotID)
{
    switch (robotID)
    {
        case RefSerialData::RobotId::RED_HERO:
        case RefSerialData::RobotId::BLUE_HERO:
            return RobotIndex::HERO;
        case RefSerialData::RobotId::RED_SOLDIER_3:
        case RefSerialData::RobotId::BLUE_SOLDIER_3:
            return RobotIndex::STANDARD;
        case RefSerialData::RobotId::RED_SENTINEL:
        case RefSerialData::RobotId::BLUE_SENTINEL:
            return RobotIndex::SENTRY;
        default:
            return RobotIndex::NUM_ROBOTS;  // Invalid
    }
}

RefSerialData::RobotId RobotOrbitStateProvider::getRobotIdFromIndex(RobotIndex index, bool isBlueTeam)
{
    switch (index)
    {
        case RobotIndex::HERO:
            return isBlueTeam ? RefSerialData::RobotId::BLUE_HERO : RefSerialData::RobotId::RED_HERO;
        case RobotIndex::STANDARD:
            return isBlueTeam ? RefSerialData::RobotId::BLUE_SOLDIER_3 : RefSerialData::RobotId::RED_SOLDIER_3;
        case RobotIndex::SENTRY:
            return isBlueTeam ? RefSerialData::RobotId::BLUE_SENTINEL : RefSerialData::RobotId::RED_SENTINEL;
        default:
            return RefSerialData::RobotId::INVALID;
    }
}

void RobotOrbitStateProvider::updateRobotState(RobotIndex index, const RobotState& state)
{
    if (index < NUM_ROBOTS)
    {
        stateEstimate.robots[index] = state;
    }
}

void RobotOrbitStateProvider::updateRobotState(
    RefSerialData::RobotId robotID,
    const RobotState& state)
{
    RobotIndex index = getRobotIndexFromId(robotID);
    if (index < NUM_ROBOTS)
    {
        stateEstimate.robots[index] = state;
    }
}

// New methods for ally robot state
void RobotOrbitStateProvider::updateAllyState(const RobotState& state)
{
    stateEstimate.allyRobot = state;
}

bool RobotOrbitStateProvider::getAllyState(RobotState& outState) const
{
    if (stateEstimate.allyRobot.valid)
    {
        outState = stateEstimate.allyRobot;
        return true;
    }
    return false;
}

RobotState RobotOrbitStateProvider::getAllyState() const
{
    return stateEstimate.allyRobot;
}

bool RobotOrbitStateProvider::getRobotState(RobotIndex index, RobotState& outState) const
{
    if (index < NUM_ROBOTS && stateEstimate.robots[index].valid)
    {
        outState = stateEstimate.robots[index];
        return true;
    }
    return false;
}

bool RobotOrbitStateProvider::getRobotState(RefSerialData::RobotId robotID, RobotState& outState) const
{
    RobotIndex index = getRobotIndexFromId(robotID);
    return getRobotState(index, outState);
}

uint8_t RobotOrbitStateProvider::getNumKnownVisionStates(
    RobotState states[MAX_TRACKED_ROBOTS]) const
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < NUM_ROBOTS; i++)
    {
        if (stateEstimate.robots[i].valid)
        {
            states[count++] = stateEstimate.robots[i];
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
