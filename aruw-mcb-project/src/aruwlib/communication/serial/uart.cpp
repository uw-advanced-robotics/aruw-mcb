/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "uart.hpp"

#include "aruwlib/rm-dev-board-a/board.hpp"

using namespace Board;

namespace aruwlib
{
namespace serial
{
bool Uart::read(UartPort port, uint8_t *data)
{
#ifdef ENV_SIMULATOR
    return false;
#else
    switch (port)
    {
        case UartPort::Uart1:
            return Usart1::read(*data);
        case UartPort::Uart2:
            return Usart2::read(*data);
        case UartPort::Uart3:
            return Usart3::read(*data);
        case UartPort::Uart6:
            return Usart6::read(*data);
        default:
            return false;
    }
#endif
}

std::size_t Uart::read(UartPort port, uint8_t *data, std::size_t length)
{
#ifdef ENV_SIMULATOR
    return 0;
#else
    switch (port)
    {
        case UartPort::Uart1:
            return Usart1::read(data, length);
        case UartPort::Uart2:
            return Usart2::read(data, length);
        case UartPort::Uart3:
            return Usart3::read(data, length);
        case UartPort::Uart6:
            return Usart6::read(data, length);
        default:
            return 0;
    }
#endif
}

std::size_t Uart::discardReceiveBuffer(UartPort port)
{
#ifdef ENV_SIMULATOR
    return 0;
#else
    switch (port)
    {
        case UartPort::Uart1:
            return Usart1::discardReceiveBuffer();
        case UartPort::Uart2:
            return Usart2::discardReceiveBuffer();
        case UartPort::Uart3:
            return Usart3::discardReceiveBuffer();
        case UartPort::Uart6:
            return Usart6::discardReceiveBuffer();
        default:
            return 0;
    }
#endif
}

bool Uart::write(UartPort port, uint8_t data)
{
#ifdef ENV_SIMULATOR
    return false;
#else
    switch (port)
    {
        case UartPort::Uart1:
            return Usart1::write(data);
        case UartPort::Uart2:
            return Usart2::write(data);
        case UartPort::Uart3:
            return Usart3::write(data);
        case UartPort::Uart6:
            return Usart6::write(data);
        default:
            return false;
    }
#endif
}

std::size_t Uart::write(UartPort port, const uint8_t *data, std::size_t length)
{
#ifdef ENV_SIMULATOR
    return 0;
#else
    switch (port)
    {
        case UartPort::Uart1:
            return Usart1::write(data, length);
        case UartPort::Uart2:
            return Usart2::write(data, length);
        case UartPort::Uart3:
            return Usart3::write(data, length);
        case UartPort::Uart6:
            return Usart6::write(data, length);
        default:
            return 0;
    }
#endif
}

bool Uart::isWriteFinished(UartPort port) const
{
#ifdef ENV_SIMULATOR
    return false;
#else
    switch (port)
    {
        case UartPort::Uart1:
            return Usart1::isWriteFinished();
        case UartPort::Uart2:
            return Usart2::isWriteFinished();
        case UartPort::Uart3:
            return Usart3::isWriteFinished();
        case UartPort::Uart6:
            return Usart6::isWriteFinished();
        default:
            return false;
    }
#endif
}

void Uart::flushWriteBuffer(UartPort port)
{
#ifndef ENV_SIMULATOR
    switch (port)
    {
        case UartPort::Uart1:
            Usart1::flushWriteBuffer();
            break;
        case UartPort::Uart2:
            Usart2::flushWriteBuffer();
            break;
        case UartPort::Uart3:
            Usart3::flushWriteBuffer();
            break;
        case UartPort::Uart6:
            Usart6::flushWriteBuffer();
            break;
        default:
            break;
    }
#endif
}

}  // namespace serial

}  // namespace aruwlib
