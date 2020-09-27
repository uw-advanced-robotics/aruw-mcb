#include "TerminalSerial.hpp"

#include <iostream>

TerminalSerial::TerminalSerial() : stream(device) {}

void TerminalSerial::update()
{
    char c;
    stream.get(c);
    if (c == '\n')
    {
        // parse the packet
        rxBuff[currLineSize] = '\0';
        currLineSize = 0;
        stream << rxBuff << modm::endl;
    }
    else
    {
        rxBuff[currLineSize++] = c;
    }
}

bool TerminalSerial::TerminalDevice::read(uint8_t &c)
{
#ifdef ENV_SIMULATOR
    std::cin >> std::noskipws >> c;
#endif
    return true;
}
bool TerminalSerial::TerminalDevice::write(uint8_t c)
{
#ifdef ENV_SIMULATOR
    std::cout << c;
#endif
    return true;
}

void TerminalSerial::TerminalDevice::flushWriteBuffer()
{
#ifdef ENV_SIMULATOR
    std::cout.flush();
#endif
}
