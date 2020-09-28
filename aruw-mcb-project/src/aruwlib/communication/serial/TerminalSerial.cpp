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

#include "TerminalSerial.hpp"

#include "aruwlib/Drivers.hpp"

namespace aruwlib
{
namespace communication
{
namespace serial
{
TerminalSerial::TerminalSerial(Drivers *drivers) : device(drivers), stream(device), drivers(drivers)
{
}

void TerminalSerial::initialize() { device.initialize(); }

void TerminalSerial::update()
{
    char c;
    stream.get(c);
    if (c == '\n')
    {
        rxBuff[currLineSize] = '\0';
        std::stringstream line;
        line << std::string(rxBuff);
        std::string header;
        line >> header;
        std::string headerStr = std::string(header);
        if (headerCallbackMap.count(headerStr) == 0)
        {
            stream << "Header \'" << header.c_str() << "\' not found" << modm::endl;
        }
        else
        {
            if (!headerCallbackMap[headerStr]->terminalSerialCallback(std::move(line), stream))
            {
                stream << "invalid arguments" << modm::endl;
            }
        }
        currLineSize = 0;
    }
    else
    {
        if (currLineSize >= MAX_LINE_LENGTH - 1)
        {
            stream << "input line too long, throwing away data" << modm::endl;
            currLineSize = 0;
        }
        else
        {
            rxBuff[currLineSize++] = c;
        }
    }
}

void TerminalSerial::addHeader(const std::string &header, ITerminalSerialCallback *callback)
{
    if (callback == nullptr)
    {
        return;
    }
    headerCallbackMap[header] = callback;
}
}  // namespace serial
}  // namespace communication
}  // namespace aruwlib
