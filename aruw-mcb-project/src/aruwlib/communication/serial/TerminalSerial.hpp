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
#include "HostedTerminalDevice.hpp"
#else
#include "UartTerminalDevice.hpp"
#endif

#include <modm/io.hpp>

#include "aruwlib/communication/serial/uart.hpp"
#include "aruwlib/rm-dev-board-a/board.hpp"

#include "mock_macros.hpp"

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
     * @param[in] streamingEnabled Set to `true` when the streaming is initially enabled.
     *      Subsequent interactions with the callback handler will be via
     *      terminalSerialStreamCallback until streaming has been disabled.
     * @return `true` if the inputLine was valid and was parsed correctly, `false` otherwise.
     */
    virtual bool terminalSerialCallback(
        std::stringstream &&inputLine,
        modm::IOStream &outputStream,
        bool streamingEnabled) = 0;

    /**
     * Called repeatedly by the TerminalSerial when in streaming mode.
     */
    virtual void terminalSerialStreamCallback(modm::IOStream &outputStream) = 0;
};  // class ITerminalSerialCallback

/**
 * Handles incoming requests from the "terminal". The "terminal" is either
 * a uart line connected to a computer with a serial connection open or
 * when runing on the simulator, stdin/stdout.
 *
 * To add a handler to the terminal, extend the ITerminalSerialCallback
 * and add it to an instance of the TerminalSerial class via `addHeader`.
 * Whenever the header is received on the terminal line, the contents of
 * the message (minus the header and any flags specified for the TerminalSerial)
 * will be passed on to the registered callback via terminalSerialCallback.
 *
 * @note If the "-S" flag is specified directly after the header, the terminal
 *      enters streaming mode. In this mode, so long as terminalSerialCallback
 *      returns true when streaming mode is initially enabled, the
 *      ITerminalSerialCallback's `terminalSerialStreamCallback` function will
 *      be called repeatedly until the user enters any new key presses. If you
 *      design an ITerminalSerialCallback that does not need/handle streaming,
 *      simply check the argument in `terminalSerialCallback` called `streamingEnabled`
 *      `return false` and write to the `outputStream` to notify the user that
 *      streaming is not enabled.
 */
class TerminalSerial
{
public:
    TerminalSerial(Drivers *drivers);

    mockable void initialize();

    mockable void update();

    mockable void addHeader(const std::string &header, ITerminalSerialCallback *callback);

private:
    static constexpr int MAX_LINE_LENGTH = 256;

    // Use either an IO device that interacts with UART or with stdin/stdout.
#ifdef PLATFORM_HOSTED
    HostedTerminalDevice device;
#else
    UartTerminalDevice device;
#endif

    ///< Hardware abstraction of a stream that provides utilities for tx/rx.
    modm::IOStream stream;

    ///< A single line is parsed into this buffer.
    char rxBuff[MAX_LINE_LENGTH];

    ///< Index into the rxBuff.
    uint8_t currLineSize = 0;

    /**
     * Used when in streaming mode to signify the serial callback that has taken
     * control of writing to the IOStream
     */
    ITerminalSerialCallback *currStreamer;

    std::map<std::string, ITerminalSerialCallback *> headerCallbackMap;

    Drivers *drivers;

    void printUsage();
};  // class TerminalSerial
}  // namespace serial
}  // namespace communication
}  // namespace aruwlib

#endif  // TERMINAL_SERIAL_HPP_
