#ifndef TERMINAL_SERIAL_HPP_
#define TERMINAL_SERIAL_HPP_

#include <cstring>
#include <map>
#include <sstream>
#ifdef ENV_SIMULATOR
#include <iostream>
#endif

#include <modm/io.hpp>

#include "aruwlib/communication/serial/uart.hpp"
#include "aruwlib/rm-dev-board-a/board.hpp"

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
    virtual std::string terminalSerialCallback(std::stringstream &&inputLine) = 0;
};  // class ITerminalSerialCallback

class TerminalSerial
{
public:
    TerminalSerial(Drivers *drivers);

    void initialize();

    void update();

    void addHeader(const std::string &header, ITerminalSerialCallback *callback);

private:
    static constexpr int MAX_LINE_LENGTH = 256;

    // TODO fix
#ifdef ENV_SIMULATOR
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
