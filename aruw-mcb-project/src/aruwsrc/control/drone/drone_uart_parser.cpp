/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include "drone_uart_parser.hpp"

#include "aruwsrc/control/drone/mavlink/common/mavlink_msg_local_position_ned.h"
#include "aruwsrc/control/drone/mavlink/mavlink_helpers.h"

namespace aruwsrc
{
namespace drone
{
DroneUartParser::DroneUartParser(Drivers *drivers) : drivers(drivers) {}

void DroneUartParser::initialize() { drivers->uart.init<DRONE_PORT, UART_BAUD_RATE>(); }

bool DroneUartParser::read(char &c)
{
    return drivers->uart.read(DRONE_PORT, &reinterpret_cast<uint8_t &>(c));
}

// Don't use this, its a read only port
void DroneUartParser::write(char c)
{
    // drivers->uart.write(DRONE_PORT, c);
}

void DroneUartParser::flush() { drivers->uart.flushWriteBuffer(DRONE_PORT); }

void DroneUartParser::compute()
{
    do
    {
        char byte;
        read(byte);
        mavlink_parse_char(COMMUNICATION_CHANNEL, byte, &currentMessage, &currentMessageStatus);
    } while (currentMessageStatus.parse_state != MAVLINK_FRAMING_OK);
    mavlink_msg_local_position_ned_decode(&currentMessage, &position);
}

}  // namespace drone
}  // namespace aruwsrc
