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

#ifndef TERMINAL_DEVICES_HPP_
#define TERMINAL_DEVICES_HPP_

#include <cstdint>
#include <iostream>

#ifdef ENV_SIMULATOR
#include <pthread.h>

#include <modm/container/deque.hpp>
#endif

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
    virtual ~HostedTerminalDevice();

    void initialize();

    bool read(char &c) override;

    using IODevice::write;
    void write(char c) override;

    void flush() override;

private:
    static constexpr int RX_BUFF_SIZE = 256;

    Drivers *drivers;

    pthread_t cinRxThread;

    pthread_mutex_t cinDataMutex;

    modm::BoundedDeque<char, RX_BUFF_SIZE> rxBuff;

    static void *readCin(void *vargp);
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
