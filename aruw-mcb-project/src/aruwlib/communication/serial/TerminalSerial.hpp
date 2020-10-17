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

#ifndef TERMINAL_SERIAL_HPP_
#define TERMINAL_SERIAL_HPP_

#include <cstring>
#include <map>
#include <sstream>
#ifdef PLATFORM_HOSTED
#include <iostream>
#endif

#include <modm/io.hpp>

#include "aruwlib/communication/serial/uart.hpp"
#include "aruwlib/rm-dev-board-a/board.hpp"

#include "mock_macros.hpp"
#include "terminal_devices.hpp"

namespace aruwlib
{
class Drivers;
namespace communication
{
namespace serial
{
/**
 * If you would like to interact with the terminal, extend this class and implement
 * the callback.
 */
class ITerminalSerialCallback
{
public:
    /**
     * @param[in] inputLine The user input to be processed.
     * @param[out] outputStream The stream to write information to.
     * @return `true` if the inputLine was valid and was parsed correctly, `false` otherwise.
     */
    virtual bool terminalSerialCallback(
        std::stringstream &&inputLine,
        modm::IOStream &outputStream) = 0;
};  // class ITerminalSerialCallback

class TerminalSerial
{
public:
    TerminalSerial(Drivers *drivers);

    mockable void initialize();

    mockable void update();

    mockable void addHeader(const std::string &header, ITerminalSerialCallback *callback);

private:
    static constexpr int MAX_LINE_LENGTH = 256;

#ifdef PLATFORM_HOSTED
    HostedTerminalDevice device;
#else
    UartTerminalDevice device;
#endif

    modm::IOStream stream;

    char rxBuff[MAX_LINE_LENGTH];

    uint8_t currLineSize = 0;

    std::map<std::string, ITerminalSerialCallback *> headerCallbackMap;

    Drivers *drivers;
};  // class TerminalSerial
}  // namespace serial
}  // namespace communication
}  // namespace aruwlib

#endif  // TERMINAL_SERIAL_HPP_
