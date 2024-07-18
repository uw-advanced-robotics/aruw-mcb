/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef MAVLINK_TELEMETRY_HPP_
#define MAVLINK_TELEMETRY_HPP_

#include "mavlink_messages.hpp"
#include "mavlink_parser.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::communication::serial::mavlink
{

class MavlinkTelemetry : public MavlinkParser
{
public:
    MavlinkTelemetry(tap::Drivers* drivers, Uart::UartPort port);

    void messageReceiveCallback(const ReceivedSerialMessage& message);

    static constexpr int MAVLINK_MSG_ID_LOCAL_POSITION_NED = 32;
    static constexpr int MAVLINK_MSG_ID_ATTITUDE = 30;

private:
    LocalPositionNED position;
    Attitude attitude;
};

}  // namespace aruwsrc::communication::serial::mavlink

#endif  // MAVLINK_TELEMETRY_HPP_
