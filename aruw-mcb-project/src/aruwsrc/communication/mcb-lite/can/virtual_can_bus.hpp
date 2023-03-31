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

#ifndef VIRTUAL_CAN_BUS_HPP_
#define VIRTUAL_CAN_BUS_HPP_

#include "tap/communication/can/can_bus.hpp"
#include "tap/communication/can/can_rx_handler.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"

#include "modm/container/queue.hpp"

#include "can_bus.hpp"

namespace aruwsrc::virtualMCB
{
template <tap::communication::serial::Uart::UartPort port>
class VirtualCanBus : public CanBus
{
public:
    VirtualCanBus(tap::Drivers* drivers);

    void initialize() override;

    bool getMessage(tap::can::CanBus canbus, modm::can::Message* message) override;

    bool isReadyToSend(tap::can::CanBus canbus) override;

    bool sendMessage(tap::can::CanBus canbus, const modm::can::Message& message) override;

private:
    tap::Drivers* drivers;

    // Blame eli if this does bad hardware kilobytes things, actually just blame him in general
    modm::BoundedQueue<modm::can::Message, 254> CAN1_queue;
    modm::BoundedQueue<modm::can::Message, 254> CAN2_queue;

    // choosing this cuz in binary this would be 10000001 so we don't read empty buffer as can bus 1
    static constexpr uint16_t CANBUS_ID_OFFSET = 129;
};

}  // namespace aruwsrc::virtualMCB

#endif