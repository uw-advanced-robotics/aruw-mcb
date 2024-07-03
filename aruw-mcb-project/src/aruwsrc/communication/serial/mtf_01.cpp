/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "mtf_01.hpp"

#define READ(data, length) drivers->uart.read(this->port, data, length)

using tap::communication::serial::Uart;

namespace aruwsrc::communication::serial
{
MTF01::MTF01(tap::Drivers *drivers, tap::communication::serial::Uart::UartPort port)
    : drivers(drivers),
      port(port)
{
}

void MTF01::initialize()
{
    switch (this->port)
    {
        case Uart::UartPort::Uart1:
            drivers->uart.init<Uart::UartPort::Uart1, BAUDRATE>();
            break;
        case Uart::UartPort::Uart2:
            drivers->uart.init<Uart::UartPort::Uart2, BAUDRATE>();
            break;
        case Uart::UartPort::Uart3:
            drivers->uart.init<Uart::UartPort::Uart3, BAUDRATE>();
            break;
        case Uart::UartPort::Uart6:
            drivers->uart.init<Uart::UartPort::Uart6, BAUDRATE>();
            break;
        case Uart::UartPort::Uart7:
            drivers->uart.init<Uart::UartPort::Uart7, BAUDRATE>();
            break;
        case Uart::UartPort::Uart8:
            drivers->uart.init<Uart::UartPort::Uart8, BAUDRATE>();
            break;
        default:
            break;
    }
}

void MTF01::read()
{
    switch (state)
    {
        case ParsingState::SERIAL_HEADER_SEARCH:
            while (state == ParsingState::SERIAL_HEADER_SEARCH && READ(&currentMessage.header, 1))
            {
                if (currentMessage.header == static_cast<uint8_t>(MTF01::ExpectedMessage::HEADER))
                {
                    state = ParsingState::PROCESS_FRAME_HEADER;
                    currentFrameIndex = 1;
                    break;
                }
                currentFrameIndex = 0;
            }
            break;
        case ParsingState::PROCESS_FRAME_HEADER:
            currentFrameIndex += READ(
                reinterpret_cast<uint8_t *>(&currentMessage) + currentFrameIndex,
                MTF01::NUM_BYTES_MESSAGE - currentFrameIndex);
            if (currentFrameIndex == MTF01::NUM_BYTES_MESSAGE)
            {
                currentFrameIndex = 0;
                bool valid = validateMessage(currentMessage);
                if (valid)
                {
                    processedMessage = currentMessage;
                }
                state = ParsingState::SERIAL_HEADER_SEARCH;
            }
            else if (currentFrameIndex >= MTF01::NUM_BYTES_MESSAGE)
            {
                currentFrameIndex = 0;
                state = ParsingState::SERIAL_HEADER_SEARCH;
            }
            break;
    }
}

};  // namespace aruwsrc::communication::serial
