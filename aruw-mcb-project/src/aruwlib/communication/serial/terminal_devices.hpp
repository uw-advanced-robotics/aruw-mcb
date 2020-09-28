#ifndef TERMINAL_DEVICES_HPP_
#define TERMINAL_DEVICES_HPP_

#include <cstdint>
#include <iostream>

#include <modm/io/iodevice.hpp>

#include "aruwlib/communication/serial/uart.hpp"

namespace aruwlib
{
class Drivers;
namespace communication
{
namespace serial
{
#ifdef ENV_SIMULATOR
/**
 * A device that interacts with stdin and stdout to be used
 * on the hosted environment
 */
class HostedTerminalDevice : public modm::IODevice
{
public:
    HostedTerminalDevice(Drivers *drivers);
    HostedTerminalDevice(const HostedTerminalDevice &) = delete;
    HostedTerminalDevice &operator=(const HostedTerminalDevice &) = delete;
    virtual ~HostedTerminalDevice() = default;

    void initialize();

    bool read(char &c) override;

    using IODevice::write;
    void write(char c) override;

    void flush() override;

private:
    Drivers *drivers;
};  // class TerminalDevice
#endif

class UartTerminalDevice : public modm::IODevice
{
public:
    UartTerminalDevice(Drivers *drivers);
    UartTerminalDevice(const UartTerminalDevice &) = delete;
    UartTerminalDevice &operator=(const UartTerminalDevice &) = delete;
    virtual ~UartTerminalDevice() = default;

    void initialize();

    bool read(char &c) override;

    using IODevice::write;
    void write(char c) override;

    void flush() override;

private:
    Drivers *drivers;

    static constexpr aruwlib::serial::Uart::UartPort TERMINAL_UART_PORT =
        aruwlib::serial::Uart::UartPort::Uart3;
};  // class UartTerminalDevice
}  // namespace serial
}  // namespace communication
}  // namespace aruwlib

#endif  // TERMINAL_DEVICES_HPP_
