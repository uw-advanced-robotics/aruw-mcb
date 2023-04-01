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
    uint8_t canbusNum;
    drivers->uart.read(port, &canbusNum);
    canbusNum -= CANBUS_ID_OFFSET;

    if (canbusNum != reinterpret_cast<uint8_t>(tap::can::CanBus::CAN_BUS1) &&
        canbusNum != reinterpret_cast<uint8_t>(tap::can::CanBus::CAN_BUS2))
    {
        return false;
    }

    modm::can::Message msg;

    bool readCompleteMessage =
        drivers->uart.read(port, reinterpret_cast<uint8_t*>(msg), sizeof(modm::can::Message)) ==
        sizeof(modm::can::Message);

    if (!readCompleteMessage)
    {
        return readCompleteMessage;
    }

    if (canbusNum == reinterpret_cast<uint8_t>(tap::can::CanBus::CAN_BUS1))
    {
        CAN1_queue.push(msg);
        msg = CAN1_queue.pop();
        memcpy(&msg, message, sizeof(modm::can::Message));
    }

    if (canbusNum == reinterpret_cast<uint8_t>(tap::can::CanBus::CAN_BUS2))
    {
        CAN2_queue.push(msg);
        msg = CAN2_queue.pop();
        memcpy(&msg, message, sizeof(modm::can::Message));
    }

    return true;
}

template <tap::communication::serial::Uart::UartPort port>
bool VirtualCanBus<port>::isReadyToSend(tap::can::CanBus canbus)
{
    return drivers->uart.isWriteFinished(port);
}

template <tap::communication::serial::Uart::UartPort port>
bool VirtualCanBus<port>::sendMessage(tap::can::CanBus canbus, const modm::can::Message& message)
{
    drivers->uart.write(port, (uint8_t)canbus + CANBUS_ID_OFFSET);
    return drivers->uart.write(
        port,
        reinterpret_cast<uint8_t*>(message),
        sizeof(modm::can::Message));
}

}  // namespace aruwsrc::virtualMCB