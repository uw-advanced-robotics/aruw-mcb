/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "mavlink_receiver.hpp"

#include "tap/errors/create_errors.hpp"

using namespace tap::communication::serial;

/**
 * Macro that wraps uart read for ease of readability in code.
 * @param[in] data Byte array where the data should be read into.
 * @param[in] length The number of bytes to read.
 * @return The number of bytes read into data.
 */
#define READ(data, length) drivers->uart.read(this->port, data, length)

namespace aruwsrc::communication::serial
{
MavlinkReceiver::MavlinkReceiver(tap::Drivers *drivers, Uart::UartPort port)
    : mavlinkSerialRxState(SERIAL_HEADER_SEARCH),
      newMessage(),
      mostRecentMessage(),
      frameCurrReadByte(0),
      drivers(drivers),
      port(port)
{
}

void MavlinkReceiver::initialize()
{
    switch (this->port)
    {
        case Uart::UartPort::Uart1:
            drivers->uart.init<Uart::UartPort::Uart1, UART_BAUDRATE>();
            break;
        case Uart::UartPort::Uart2:
            drivers->uart.init<Uart::UartPort::Uart2, UART_BAUDRATE>();
            break;
        case Uart::UartPort::Uart3:
            drivers->uart.init<Uart::UartPort::Uart3, UART_BAUDRATE>();
            break;
        case Uart::UartPort::Uart6:
            drivers->uart.init<Uart::UartPort::Uart6, UART_BAUDRATE>();
            break;
        case Uart::UartPort::Uart7:
            drivers->uart.init<Uart::UartPort::Uart7, UART_BAUDRATE>();
            break;
        case Uart::UartPort::Uart8:
            drivers->uart.init<Uart::UartPort::Uart8, UART_BAUDRATE>();
            break;
        default:
            break;
    }
}

void MavlinkReceiver::updateSerial()
{
    reading++;
    switch (mavlinkSerialRxState)
    {
        case SERIAL_HEADER_SEARCH:
        {
            // keep scanning for the head byte as long as you are here and have not yet found it.
            while (mavlinkSerialRxState == SERIAL_HEADER_SEARCH &&
                   READ(&newMessage.header.headByte, 1))
            {
                readFromUart++;
                // we found it, store the head byte
                if (newMessage.header.headByte == SERIAL_HEAD_BYTE)
                {
                    mavlinkSerialRxState = PROCESS_FRAME_HEADER;
                    frameCurrReadByte = 0;
                    headBytesCorrect++;
                }
            }
            break;
        }
        case PROCESS_FRAME_HEADER:  // the frame header consists of the length, seq, and message id
        {
            settingHeader++;
            frameCurrReadByte += READ(
                reinterpret_cast<uint8_t *>(&newMessage) + frameCurrReadByte + 1,
                sizeof(newMessage.header) - frameCurrReadByte - 1);

            // We have the complete message header in the frameHeader buffer
            if (frameCurrReadByte == (sizeof(newMessage.header) - 1))
            {
                frameCurrReadByte = 0;

                if (newMessage.header.messageId == 32)
                {
                    gotaThirtyTwoMessageID++;
                }

                datathingy[newMessage.header.messageId]++;

                // move on to processing message body
                mavlinkSerialRxState = PROCESS_FRAME_DATA;
            }
            break;
            case PROCESS_FRAME_DATA:  // READ bulk of message
            {
                bytesToRead = sizeof(newMessage.CRC16) + newMessage.header.dataLength;

                frameCurrReadByte += READ(
                    reinterpret_cast<uint8_t *>(&newMessage) + sizeof(newMessage.header) +
                        frameCurrReadByte,
                    bytesToRead - frameCurrReadByte);

                if (frameCurrReadByte == bytesToRead)
                {
                    // move crc16 to `CRC16` position (currently in the data section if <
                    // SERIAL_RX_BUFF_SIZE length sent)
                    memcpy(
                        &newMessage.CRC16,
                        newMessage.data + newMessage.header.dataLength,
                        sizeof(newMessage.CRC16));

                    readAFullMessage++;

                    tempCRC = crc_calculate(
                        reinterpret_cast<uint8_t *>(&(newMessage.header)) + 1,
                        5);  // Calculate for header first

                    crc_accumulate_buffer(
                        &tempCRC,
                        reinterpret_cast<const char *>(newMessage.data),
                        newMessage.header.dataLength);  // Then calculate for data
                    crc_accumulate(CRC_EXTRA[newMessage.header.messageId], &tempCRC);

                    if (newMessage.CRC16 != tempCRC)
                    {
                        mavlinkSerialRxState = SERIAL_HEADER_SEARCH;
                        RAISE_ERROR(drivers, "CRC16 failure");
                        failedCRC++;
                        return;
                    }

                    mostRecentMessage = newMessage;

                    messageReceiveCallback(mostRecentMessage);

                    mavlinkSerialRxState = SERIAL_HEADER_SEARCH;
                }
                else if (frameCurrReadByte > bytesToRead)
                {
                    frameCurrReadByte = 0;
                    RAISE_ERROR(drivers, "Invalid message length");
                    readTooMuch++;
                    mavlinkSerialRxState = SERIAL_HEADER_SEARCH;
                }
                break;
            }
        }
    }
}

}  // namespace aruwsrc::communication::serial
