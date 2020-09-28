#include "terminal_devices.hpp"

#include "aruwlib/Drivers.hpp"

namespace aruwlib
{
namespace communication
{
namespace serial
{
#ifdef ENV_SIMULATOR
HostedTerminalDevice::HostedTerminalDevice(Drivers *drivers) : drivers(drivers) {}

void HostedTerminalDevice::initialize()
{
    // TODO set up background pthread that uses cin to read from terminal
}

bool HostedTerminalDevice::read(char &c)
{
    // TODO: Make non-blocking (use background thread)
    std::cin >> std::noskipws >> c;
    return true;
}

void HostedTerminalDevice::write(char c) { std::cout << c; }

void HostedTerminalDevice::flush() { std::cout.flush(); }
#endif

UartTerminalDevice::UartTerminalDevice(Drivers *drivers) : drivers(drivers) {}

void UartTerminalDevice::initialize() { drivers->uart.init<aruwlib::serial::Uart::Uart3, 9600>();}

bool UartTerminalDevice::read(char &c)
{
    return drivers->uart.read(TERMINAL_UART_PORT, &reinterpret_cast<uint8_t &>(c));
}

void UartTerminalDevice::write(char c) { drivers->uart.write(TERMINAL_UART_PORT, c); }

void UartTerminalDevice::flush() { drivers->uart.flushWriteBuffer(TERMINAL_UART_PORT); }
}  // namespace serial
}  // namespace communication
}  // namespace aruwlib
