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

#ifndef ROBOT_SHARE_MESSAGE_TYPES_HPP_
#define ROBOT_SHARE_MESSAGE_TYPES_HPP_

#include <cinttypes>

namespace aruwsrc::communication::serial
{

static constexpr uint16_t TARGET_SHARE_ROBOT_ID = 0x202;  // should i just copy request and do 200

enum class RobotShareMessageType : uint8_t
{
    MESSAGE = 0,
    NUM_MESSAGE_TYPES,
};
}  // namespace aruwsrc::communication::serial

#endif  //  ROBOT_SHARE_MESSAGE_TYPES_HPP_
