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
            stream << headerCallbackMap[headerStr]->terminalSerialCallback(std::move(line)).c_str()
                   << modm::endl;
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
