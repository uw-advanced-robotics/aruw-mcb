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

#include "virtual_can_bus.hpp"

#include "tap/communication/can/can.hpp"
#include "tap/communication/can/can_bus.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"

namespace aruwsrc::virtualMCB
{

template <tap::communication::serial::Uart::UartPort port>
VirtualCanBus<port>::VirtualCanBus(tap::Drivers* drivers) : drivers(drivers)
{
}

template <tap::communication::serial::Uart::UartPort port>
void VirtualCanBus<port>::initialize()
{
    drivers->uart.init<port, 1'000'000>();
}

template <tap::communication::serial::Uart::UartPort port>
bool VirtualCanBus<port>::getMessage(tap::can::CanBus canbus, modm::can::Message* message)
{
    // Get which canbus the message is from
    uint8_t canbusNum;
    drivers->uart.read(port, &canbusNum);
    canbusNum -= CANBUS_ID_OFFSET;

    // Check if the canbus number is valid
    if (canbusNum != static_cast<uint8_t>(tap::can::CanBus::CAN_BUS1) &&
        canbusNum != static_cast<uint8_t>(tap::can::CanBus::CAN_BUS2))
    {
        return false;
    }

    // Read the message
    modm::can::Message msg;
    uint8_t* holder = new uint8_t[sizeof(modm::can::Message)];
    uint8_t readSize = drivers->uart.read(port, holder, sizeof(modm::can::Message));
    memcpy(&msg, holder, sizeof(modm::can::Message));

    if (readSize != sizeof(modm::can::Message))
    {
        return false;
    }

    // Push the message to the correct queue
    if (canbusNum == static_cast<uint8_t>(tap::can::CanBus::CAN_BUS1))
    {
        CAN1_queue.push(msg);
    }
    else
    {
        CAN2_queue.push(msg);
    }

    // Check if the message is which canbus
    if (canbus == tap::can::CanBus::CAN_BUS1)
    {
        msg = CAN1_queue.get();
    }
    else
    {
        msg = CAN2_queue.get();
    }

    memcpy(&msg, message, sizeof(modm::can::Message));

    return true;
}

template <tap::communication::serial::Uart::UartPort port>
bool VirtualCanBus<port>::isReadyToSend()
{
    return drivers->uart.isWriteFinished(port);
}

template <tap::communication::serial::Uart::UartPort port>
bool VirtualCanBus<port>::sendMessage(tap::can::CanBus canbus, const modm::can::Message& message)
{
    drivers->uart.write(port, (uint8_t)canbus + CANBUS_ID_OFFSET);
    uint16_t* holder = new uint16_t[sizeof(modm::can::Message)];
    memcpy(holder, &message, sizeof(modm::can::Message));
    return drivers->uart.write(
        port,
        reinterpret_cast<uint8_t*>(holder),
        sizeof(modm::can::Message));
}

}  // namespace aruwsrc::virtualMCB