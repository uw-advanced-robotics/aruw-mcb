#ifndef TERMINAL_SERIAL_HPP_
#define TERMINAL_SERIAL_HPP_

#include <modm/io.hpp>

class TerminalSerial
{
public:
    TerminalSerial();

    void update();

    // modm::IODeviceWrapper<SerialDevice, modm::IOBuffer::DiscardIfFull> serialDevice;
    // serialDevice.write('f');
    // char c;
    class TerminalDevice
    {
    public:
        static bool read(uint8_t &c);
        static bool write(uint8_t c);
        static void flushWriteBuffer();
    };  // class TerminalDevice

private:
    modm::IODeviceWrapper<TerminalDevice, modm::IOBuffer::DiscardIfFull> device;
    modm::IOStream stream;
    char rxBuff[100];
    uint8_t currLineSize = 0;
};  // class TerminalSerial

#endif  // TERMINAL_SERIAL_HPP_
