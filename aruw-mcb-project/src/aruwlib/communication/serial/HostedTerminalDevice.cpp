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

#ifdef PLATFORM_HOSTED

#include "HostedTerminalDevice.hpp"

#include <iostream>

namespace aruwlib
{
namespace communication
{
namespace serial
{
HostedTerminalDevice::HostedTerminalDevice(Drivers *drivers)
    : drivers(drivers),
      readStdinThread(&HostedTerminalDevice::readCin, this),
      rxBuff(),
      rxBuffMutex()
{
}

void HostedTerminalDevice::readCin()
{
    while (true)
    {
        char c;
        ::std::cin >> ::std::noskipws >> c;
        rxBuffMutex.lock();
        rxBuff.appendOverwrite(c);
        rxBuffMutex.unlock();
    }
}

void HostedTerminalDevice::initialize() { readStdinThread.detach(); }

bool HostedTerminalDevice::read(char &c)
{
    rxBuffMutex.lock();
    if (rxBuff.getSize() > 0)
    {
        c = rxBuff.getFront();
        rxBuff.removeFront();

        rxBuffMutex.unlock();
        return true;
    }
    rxBuffMutex.unlock();
    return false;
}

void HostedTerminalDevice::write(char c) { ::std::cout << c; }

void HostedTerminalDevice::flush() { ::std::cout.flush(); }
}  // namespace serial
}  // namespace communication
}  // namespace aruwlib

#endif
