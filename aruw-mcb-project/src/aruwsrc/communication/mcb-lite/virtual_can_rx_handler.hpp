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

#include "can_bus.hpp"
#include "motor/virtual_dji_motor_tx_handler.hpp"

#include "tap/communication/can/can_rx_handler.hpp"

namespace aruwsrc::can
{
class VirtualCANRxHandler : public tap::can::CanRxHandler
{

VirtualCANRxHandler()


};

}  // namespace aruwsrc::can

#endif