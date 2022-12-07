/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/communication/serial/uart.hpp"
#include "taproot/modm/src/modm/io/iodevice.hpp"

#include "aruwsrc/drivers.hpp"


using namespace tap::communication::serial;


namespace aruwsrc
{
namespace drone
{

class DroneUartParser : public ::modm::IODevice
{
public:
    DroneUartParser(Drivers *drivers);

    void initialize();

    bool read(char &c) override;

    void write(char c) override;

    void flush() override;

private:
    // TODO: this is what is used in terminal device, need to figure out what this should be
    static constexpr uint32_t UART_BAUD_RATE = 115200;

    Drivers *drivers;

    static constexpr Uart::UartPort DRONE_PORT = Uart::Uart1;
};
}  // namespace drone
}  // namespace aruwsrc
