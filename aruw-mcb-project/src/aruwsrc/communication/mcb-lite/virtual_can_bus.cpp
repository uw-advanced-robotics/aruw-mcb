/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/communication/can/can.hpp"
#include "tap/communication/can/can_bus.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"

#include "virtual_can_bus.hpp"

namespace aruwsrc::virtualMCB
{

VirtualCanBus::VirtualCanBus(tap::Drivers* drivers, tap::communication::serial::Uart::UartPort port)
    : drivers(drivers),
      port(port)
{
}

void VirtualCanBus::initialize()
{
    // drivers->uart.init<tap::communication::serial::Uart::Port::Uart3, 1000000>();
    // drivers->uart.init<port, 1000000>();
}

bool VirtualCanBus::isMessageAvailable(tap::can::CanBus canbus)
{
    // Uuuuuuuuh this doesn't really exist on the uart port?? Also TODO: Where is this used???
    //  drivers->uart.i
}

bool VirtualCanBus::getMessage(tap::can::CanBus canbus, modm::can::Message* message)
{
    uint32_t* messageCast;
    // This is read at 32 for can message??
    uint32_t readSize = drivers->uart.read(port, reinterpret_cast<uint8_t*>(messageCast), sizeof(message));
    memcpy(&message, &messageCast, sizeof(message));
    // TODO: Figure out if this is correct

}

bool VirtualCanBus::isReadyToSend(tap::can::CanBus canbus)
{
    drivers->uart.isWriteFinished(port);
}

bool VirtualCanBus::sendMessage(tap::can::CanBus canbus, const modm::can::Message& message)
{
    // Copy of what's done in the can send message
    uint32_t* messageCast;
    memcpy(&messageCast, &message, sizeof(message));
    drivers->uart.write(port, reinterpret_cast<uint8_t*>(messageCast), sizeof(message));
}

}  // namespace aruwsrc::can