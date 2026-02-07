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

#ifndef RTT_CONNECTION_STATE_HPP_
#define RTT_CONNECTION_STATE_HPP_

namespace aruwsrc::communication::rtt
{
// Enum to track state of RTT connection
enum class ConnectionState : int
{
    Ozone = 0,
    Unidrictional = 1,
    Bidirectional = 2,
};

constexpr const char* connectionStateNames[] = {
    "Ozone",
    "Unidirectional",
    "Bidirectional",
};

inline const char* connectionStateToString (ConnectionState state)
{
    return connectionStateNames[static_cast<int>(state)];
}

}  // namespace aruwsrc::communication::rtt

#endif  // RTT_CONNECTION_STATE_HPP_