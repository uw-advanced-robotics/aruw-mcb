/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SENTINEL_REQUEST_MESSAGE_TYPES_HPP_
#define SENTINEL_REQUEST_MESSAGE_TYPES_HPP_

#include <cinttypes>

namespace aruwsrc::communication::serial
{
static constexpr uint16_t SENTINEL_REQUEST_ROBOT_ID = 0x200;

enum class SentinelRequestMessageType : uint8_t
{
    SELECT_NEW_ROBOT = 0,
    TARGET_NEW_QUADRANT,
    TOGGLE_DRIVE_MOVEMENT,
    NUM_MESSAGE_TYPES,
};
}  // namespace aruwsrc::communication::serial

#endif  //  SENTINEL_REQUEST_MESSAGE_TYPES_HPP_
