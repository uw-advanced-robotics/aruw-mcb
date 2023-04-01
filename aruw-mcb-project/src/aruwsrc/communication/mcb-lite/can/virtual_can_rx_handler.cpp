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

namespace aruwsrc::virtualMCB
{
VirtualCANRxHandler::VirtualCANRxHandler(tap::Drivers* drivers, CanBus* canbus)
    : CanRxHandler(drivers),
      canbus(canbus)
{
}

void VirtualCANRxHandler::pollCanData()
{
    modm::can::Message rxMessage;

    // handle incoming CAN 1 messages
    if (canbus->getMessage(tap::can::CanBus::CAN_BUS1, &rxMessage))
    {
        CanRxHandler::processReceivedCanData(rxMessage, messageHandlerStoreCan1);
    }

    // handle incoming CAN 2 messages
    if (canbus->getMessage(tap::can::CanBus::CAN_BUS2, &rxMessage))
    {
        CanRxHandler::processReceivedCanData(rxMessage, messageHandlerStoreCan2);
    }
}

}  // namespace aruwsrc::virtualMCB