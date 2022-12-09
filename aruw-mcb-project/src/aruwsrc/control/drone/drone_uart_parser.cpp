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

bool DroneUartParser::read(auto &c, int length)
{
    return drivers->uart.read(DRONE_PORT, &reinterpret_cast<uint8_t &>(c), length);
}

/**
 * Definition of protocol:
 * 1st byte: Start
 * 2nd byte: Payload size
 * 3rd byte: Packet number (used to detect packet loss)
 * 4th byte: System ID (Should be 0, id of drone)
 * 5th byte: Component ID (Part of drone sending it)
 * 6th byte: Message ID (what type of message being sent)
 * The next 24 bytes: The payload (actual data we care about)
 * The next 3 bytes: CRC
 */
MessageStatus DroneUartParser::update()
{
    char messageStart;
    read(messageStart);
    if (messageStart != MSG_START)
    {
        return MessageStatus::INCORRECT_START;
    }
    char messageLength;
    read(messageLength);
    if (messageLength != MSG_PAYLOAD_LENGTH)
    {
        return MessageStatus::INCORRECT_PAYLOAD_LENGTH;
    }
    char packetCount;
    read(packetCount);
    char systemID;
    read(systemID);
    char messageID;
    read(messageID);
    if (messageID != MSG_ID)
    {
        return MessageStatus::INCORRECT_MESSAGE_ID;
    }
    read(telemetryData.timestamp, 4);
    read(telemetryData.x, 4);
    read(telemetryData.y, 4);
    read(telemetryData.z, 4);
    read(telemetryData.vx, 4);
    read(telemetryData.vy, 4);
    read(telemetryData.vz, 4);
    uint16_t checksum;
    read(checksum, 2);
    if(checksum != MSG_CRC){
        return MessageStatus::INCORRECT_CRC;
    }
    return MessageStatus::OK;
}

}  // namespace drone
}  // namespace aruwsrc
