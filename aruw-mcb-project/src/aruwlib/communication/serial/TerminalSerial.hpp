#ifndef TERMINAL_SERIAL_HPP_
#define TERMINAL_SERIAL_HPP_

#include <cstring>
#include <sstream>
#include <map>
#ifdef ENV_SIMULATOR
#include <iostream>
#endif

#include <modm/io.hpp>
#include "aruwlib/rm-dev-board-a/board.hpp"
#include "aruwlib/communication/serial/uart.hpp"

namespace aruwlib
{
namespace communication
{
namespace serial
{
#ifdef ENV_SIMULATOR
class HostedTerminalDevice
{
public:
    static bool read(uint8_t &c)
    {
        std::cin >> std::noskipws >> c;
        return true;
    }

    static bool write(uint8_t c)
    {
        std::cout << c;
        return true;
    }

    static void flushWriteBuffer() { std::cout.flush(); }
};  // class TerminalDevice
#endif

class ITerminalSerialCallback
{
public:
    virtual std::string terminalSerialCallback(std::stringstream &&inputLine) = 0;
};  // class ITerminalSerialCallback

template <typename SerialDevice>
class TerminalSerial
{
public:
    typedef std::string (*TerminalSerialCallback)(std::stringstream &&inputLine);

    TerminalSerial() : stream(device) {}

    template<modm::baudrate_t baudrate, aruwlib::serial::Uart::Parity parity = aruwlib::serial::Uart::Parity::Disabled>
    void initialize()
    {
        #ifndef ENV_SIMULATOR
        SerialDevice::template connect<GpioD8::Tx, GpioD9::Rx>();
        SerialDevice::template initialize<Board::SystemClock, baudrate>(12, parity);
        #endif
    }

    void update()
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
                stream
                    << headerCallbackMap[headerStr]->terminalSerialCallback(std::move(line)).c_str()
                    << modm::endl;
            }
            currLineSize = 0;
        }
        else
        {
            rxBuff[currLineSize++] = c;
        }
    }

    void addHeader(const std::string &header, ITerminalSerialCallback *callback)
    {
        if (callback == nullptr)
        {
            return;
        }
        headerCallbackMap[header] = callback;
    }

private:
    static constexpr int MAX_LINE_LENGTH = 256;

    modm::IODeviceWrapper<SerialDevice, modm::IOBuffer::DiscardIfFull> device;
    modm::IOStream stream;
    char rxBuff[MAX_LINE_LENGTH];
    uint8_t currLineSize = 0;
    std::map<std::string, ITerminalSerialCallback *> headerCallbackMap;
};  // class TerminalSerial
}  // namespace serial
}  // namespace communication
}  // namespace aruwlib

#endif  // TERMINAL_SERIAL_HPP_
