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

#include "terminal_devices.hpp"

#include "aruwlib/Drivers.hpp"

namespace aruwlib
{
namespace communication
{
namespace serial
{
#ifdef ENV_SIMULATOR
HostedTerminalDevice::HostedTerminalDevice(Drivers *drivers) : drivers(drivers) {}

void HostedTerminalDevice::initialize()
{
    // TODO set up background pthread that uses cin to read from terminal
}

bool HostedTerminalDevice::read(char &c)
{
    // TODO: Make non-blocking (use background thread)
    std::cin >> std::noskipws >> c;
    return true;
}

void HostedTerminalDevice::write(char c) { std::cout << c; }

void HostedTerminalDevice::flush() { std::cout.flush(); }
#endif

UartTerminalDevice::UartTerminalDevice(Drivers *drivers) : drivers(drivers) {}

void UartTerminalDevice::initialize() { drivers->uart.init<aruwlib::serial::Uart::Uart3, 9600>(); }

bool UartTerminalDevice::read(char &c)
{
    return drivers->uart.read(TERMINAL_UART_PORT, &reinterpret_cast<uint8_t &>(c));
}

void UartTerminalDevice::write(char c) { drivers->uart.write(TERMINAL_UART_PORT, c); }

void UartTerminalDevice::flush() { drivers->uart.flushWriteBuffer(TERMINAL_UART_PORT); }
}  // namespace serial
}  // namespace communication
}  // namespace aruwlib
