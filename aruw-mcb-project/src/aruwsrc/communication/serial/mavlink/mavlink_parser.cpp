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
      state(HEADER_SEARCH),
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
    switch (state)
    {
        case HEADER_SEARCH:
        {
            while (state == HEADER_SEARCH && READ(&newMessage.header.frame_head_byte, 1))
            {
                if (newMessage.header.frame_head_byte == HEAD_BYTE)
                {
                    state = PROCESSING_FRAME_HEADER;
                    currByte = 1;
                    foundHeadByte++;
                }
            }
            break;
        }
        case PROCESSING_FRAME_HEADER:
        {
            currByte += READ(
                reinterpret_cast<uint8_t*>(&newMessage) + currByte,
                sizeof(newMessage.header) - currByte);
            if (currByte == sizeof(newMessage.header))
            {
                std::cout << "Finished reading header" << std::endl;
                std::cout << "Last value is " << static_cast<int>(newMessage.header.msgid_value())
                          << std::endl;

                readAllOfAHeader++;
                currByte = 0;
                state = PROCESS_PAYLOAD_AND_CRC;
            }
            break;
        }
        case PROCESS_PAYLOAD_AND_CRC:
        {
            int desired_length = newMessage.header.payload_len + sizeof(newMessage.crc);

            std::cout << "Reading payload for length: "
                      << desired_length << " currByte is "
                      << currByte << std::endl;

            currByte += READ(
                reinterpret_cast<uint8_t*>(&newMessage.payload) + currByte,
                desired_length - currByte);

            if (currByte == desired_length)
            {
                std::cout << "Read a whole message" << std::endl;
                // Print it out in hex
                for (size_t i = 0; i < sizeof(newMessage); i++)
                {
                    std::cout << std::hex << (int)*((uint8_t*)&newMessage + i) << " ";
                }
                std::cout << std::dec << std::endl;

                readAWholeMessage++;
                state = HEADER_SEARCH;
                currByte = 0;

                // Move and check CRC
                memcpy(
                    &newMessage.crc,
                    &newMessage.payload[newMessage.header.payload_len],
                    sizeof(newMessage.crc));
                if (!validateCRC(newMessage))
                {
                    CRCFailed++;
                    break;
                }
                messageReceiveCallback(newMessage);
                mostRecentMessage = newMessage;
            }
            break;
        }
    }
}

bool MavlinkParser::validateCRC(ReceivedSerialMessage& message)
{
    std::cout << "Validating CRC of len: "
              << sizeof(message.header) - 1 + message.header.payload_len << std::endl;
    uint16_t crc = crc_calculate(
        (uint8_t*)&message.header + 1,
        sizeof(message.header) - 1 + message.header.payload_len);

    std::cout << "CRC value before the magic number: " << crc << std::endl;

    uint8_t crc_extra = get_crc_extra(message.header.msgid_value());
    std::cout << "The CRC extra is: " << static_cast<int>(crc_extra) << std::endl;
    crc_accumulate(crc_extra, &crc);
    std::cout << "Calculated CRC: " << crc << " Message CRC: " << message.crc << std::endl;
    return crc == message.crc;
}

}  // namespace aruwsrc::communication::serial::mavlink
