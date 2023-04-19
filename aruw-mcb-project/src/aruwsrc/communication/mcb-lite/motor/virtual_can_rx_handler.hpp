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

#ifndef VIRTUAL_CAN_RX_HANDLER_HPP_
#define VIRTUAL_CAN_RX_HANDLER_HPP_

#include "tap/communication/can/can_rx_handler.hpp"
#include "tap/communication/can/can_rx_listener.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "modm/architecture/interface/assert.h"
#include "modm/architecture/interface/can_message.hpp"

namespace aruwsrc::virtualMCB
{
/**
 * This class is used to pass CAN messages from the virtual MCB to listeners (motors)
 */
class VirtualCanRxHandler : public tap::can::CanRxHandler
{
public:
    VirtualCanRxHandler(tap::Drivers* drivers);

    void refresh(tap::can::CanBus canbus, modm::can::Message message);

    uint32_t can1Data = 0;
    uint32_t can2Data = 0;
};

}  // namespace aruwsrc::virtualMCB

#endif
