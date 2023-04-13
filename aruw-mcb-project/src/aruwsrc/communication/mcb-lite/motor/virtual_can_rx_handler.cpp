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

#include "virtual_can_rx_handler.hpp"

#include "tap/communication/can/can.hpp"
#include "tap/communication/can/can_bus.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

namespace aruwsrc::virtualMCB
{
VirtualCanRxHandler::VirtualCanRxHandler(tap::Drivers* drivers)
    : CanRxHandler(drivers),
      messageHandlersCan1(),
      messageHandlersCan2()
{
}

void VirtualCanRxHandler::refresh(tap::can::CanBus canbus, modm::can::Message message)
{
    if (canbus == tap::can::CanBus::CAN_BUS1)
    {
        processTheReceivedCanData(message, messageHandlersCan1);
    }
    else
    {
        processTheReceivedCanData(message, messageHandlersCan2);
    }
}

void VirtualCanRxHandler::attachReceiveHandler(tap::can::CanRxListener* const listener)
{
    if (listener->canBus == tap::can::CanBus::CAN_BUS1)
    {
        attachReceiveHandler(listener, messageHandlersCan1);
    }
    else
    {
        attachReceiveHandler(listener, messageHandlersCan2);
    }
}

void VirtualCanRxHandler::attachReceiveHandler(
    tap::can::CanRxListener* const canRxListener,
    tap::can::CanRxListener** messageHandlerStore)
{
    uint16_t id = lookupTableIndexForCanId(canRxListener->canIdentifier);

    modm_assert(id < NUM_CAN_IDS, "CAN", "RX listener id out of bounds", 1);
    modm_assert(messageHandlerStore[id] == nullptr, "CAN", "overloading", 1);

    messageHandlerStore[id] = canRxListener;
}

void VirtualCanRxHandler::processTheReceivedCanData(
    const modm::can::Message& rxMessage,
    tap::can::CanRxListener* const* messageHandlerStore)
{
    uint16_t id = lookupTableIndexForCanId(rxMessage.getIdentifier());

    if (id >= NUM_CAN_IDS)
    {
        RAISE_ERROR(drivers, "Invalid can id received");
        return;
    }

    if (messageHandlerStore[id] != nullptr)
    {
        messageHandlerStore[id]->processMessage(rxMessage);
    }
}

}  // namespace aruwsrc::virtualMCB