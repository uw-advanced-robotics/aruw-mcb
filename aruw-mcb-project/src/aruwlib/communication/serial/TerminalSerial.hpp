#ifndef TERMINAL_SERIAL_HPP_
#define TERMINAL_SERIAL_HPP_

#include <cstring>
#include <map>
#ifdef ENV_SIMULATOR
#include <iostream>
#endif

#include <modm/io.hpp>

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

template <typename SerialDevice>
class TerminalSerial
{
public:
    typedef void (*TerminalSerialCallback)(char *);

    TerminalSerial() : stream(device) {}

    void update()
    {
        char c;
        stream.get(c);
        if (c == '\n')
        {
            rxBuff[currLineSize] = '\0';
            std::string header = std::string(strtok(rxBuff, " "));
            if (headerCallbackMap.count(header) == 0)
            {
                stream << "Header \'" << header.c_str() << "\' not found" << modm::endl;
            }
            else
            {
                headerCallbackMap[header](rxBuff);
            }
            currLineSize = 0;
        }
        else
        {
            rxBuff[currLineSize++] = c;
        }
    }

    void addHeader(const std::string &header, TerminalSerialCallback callback)
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
    std::map<std::string, TerminalSerialCallback> headerCallbackMap;
};  // class TerminalSerial
}  // namespace serial
}  // namespace communication
}  // namespace aruwlib

#endif  // TERMINAL_SERIAL_HPP_
