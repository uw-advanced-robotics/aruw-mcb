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

#ifndef ROBOT_ORBIT_STATE_HPP_
#define ROBOT_ORBIT_STATE_HPP_

#include <cstdint>

#include "tap/communication/serial/ref_serial_data.hpp"

using namespace tap::communication::serial;
namespace aruwsrc::communication::serial
{
constexpr uint8_t MAX_TRACKED_ROBOTS = 4;

struct RobotState
{
    RefSerialData::RobotId robotId;
    uint8_t xPos;
    uint8_t yPos;
    uint8_t zPos;
    uint32_t timestamp;
};

class RobotOrbitStateProvider
{
public:
    void updateFromVision(RefSerialData::RobotId robotID, const RobotState& state);
    void updateFromAlly(RefSerialData::RobotId robotID, const RobotState& state);
    bool getRobotState(RefSerialData::RobotId robotID, RobotState& outState) const;
    uint8_t getNumKnownVisionStates(RobotState states[MAX_TRACKED_ROBOTS]) const;
    RobotState getRobotState(RefSerialData::RobotId robotID) const;

private:
    RobotState storedStates[MAX_TRACKED_ROBOTS] = {};
    RefSerialData::RobotId storedIDs[MAX_TRACKED_ROBOTS] = {RefSerialData::RobotId::INVALID};
    bool hasState[MAX_TRACKED_ROBOTS] = {false};

    int findRobotIndex(RefSerialData::RobotId robotID) const;
};

}  // namespace aruwsrc::communication::serial

#endif  // ROBOT_ORBIT_STATE_HPP_
