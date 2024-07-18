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

#include "mavlink_parser.hpp"

#include <iostream>

using namespace tap::communication::serial;

/**
 * Macro that wraps uart read for ease of readability in code.
 * @param[in] data Byte array where the data should be read into.
 * @param[in] length The number of bytes to read.
 * @return The number of bytes read into data.
 */
#define READ(data, length) drivers->uart.read(this->port, data, length)

namespace aruwsrc::communication::serial::mavlink
{
MavlinkParser::MavlinkParser(tap::Drivers* drivers, Uart::UartPort port)
    : drivers(drivers),
      port(port),
      newMessage(),
      mostRecentMessage()
{
}

void MavlinkParser::initialize()
{
    switch (this->port)
    {
        case Uart::UartPort::Uart1:
            drivers->uart.init<Uart::UartPort::Uart1, BAUD_RATE>();
            break;
        case Uart::UartPort::Uart2:
            drivers->uart.init<Uart::UartPort::Uart2, BAUD_RATE>();
            break;
        case Uart::UartPort::Uart3:
            drivers->uart.init<Uart::UartPort::Uart3, BAUD_RATE>();
            break;
        case Uart::UartPort::Uart6:
            drivers->uart.init<Uart::UartPort::Uart6, BAUD_RATE>();
            break;
        case Uart::UartPort::Uart7:
            drivers->uart.init<Uart::UartPort::Uart7, BAUD_RATE>();
            break;
        case Uart::UartPort::Uart8:
            drivers->uart.init<Uart::UartPort::Uart8, BAUD_RATE>();
            break;
        default:
            break;
    }
}

void MavlinkParser::read()
{
    startedParsing++;
    std::cout << "Attempting a read, state is " << state << std::endl;

    switch (state)
    {
        case HEADER_SEARCH:
            while (state == HEADER_SEARCH && READ(&newMessage.header.frame_head_byte, 1))
            {
                std::cout << "The read value is " << newMessage.header.frame_head_byte << std::endl;
                if (newMessage.header.frame_head_byte == HEAD_BYTE)
                {
                    state = PROCESSING_FRAME_HEADER;
                    currByte = 1;
                    foundHeadByte++;
                }
            }
            break;
        case PROCESSING_FRAME_HEADER:
            currByte += READ(
                reinterpret_cast<uint8_t*>(&newMessage) + currByte,
                sizeof(newMessage.header) - currByte);
            if (currByte == sizeof(newMessage.header))
            {
                readAllOfAHeader++;
                currByte = 0;
                if (newMessage.header.payload_len > MAX_PAYLOAD_SIZE)
                {
                    PayloadTooBig++;
                    state = HEADER_SEARCH;
                    break;
                }

                state = PROCESS_PAYLOAD;
            }
            break;
        case PROCESS_PAYLOAD:
            currByte += READ(
                reinterpret_cast<uint8_t*>(&newMessage.payload) + currByte,
                newMessage.header.payload_len - currByte);
            if (currByte == newMessage.header.payload_len)
            {
                state = PROCESS_CRC;
                currByte = 0;
                readAWholePayload++;
            }
            break;
        case PROCESS_CRC:
            currByte += READ(
                reinterpret_cast<uint8_t*>(&newMessage.crc) + currByte,
                sizeof(newMessage.crc) - currByte);
            if (currByte == sizeof(newMessage.crc))
            {
                readAWholeMessage++;
                if (!validateCRC(newMessage))
                {
                    CRCFailed++;
                    state = HEADER_SEARCH;
                    currByte = 0;
                    break;
                }
                messageReceiveCallback(newMessage);
                mostRecentMessage = newMessage;
                state = HEADER_SEARCH;
                currByte = 0;
            }
            break;
        default:
            break;
    }
}

bool MavlinkParser::validateCRC(ReceivedSerialMessage& message)
{
    uint16_t crc = crc_calculate(
        (uint8_t*)&message + 1,
        sizeof(message.header) - 1 + message.header.payload_len);
    uint8_t crc_extra = get_crc_extra(message.header.msgid);
    crc_accumulate(crc_extra, &crc);
    return crc == message.crc;
}

}  // namespace aruwsrc::communication::serial::mavlink
