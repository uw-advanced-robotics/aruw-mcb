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

#ifndef VIRTUAL_MCB_HANDLER_HPP_
#define VIRTUAL_MCB_HANDLER_HPP_

#include "can_bus.hpp"
#include "motor/virtual_dji_motor_tx_handler.hpp"
#include "virtual_can_rx_handler.hpp"

namespace aruwsrc::virtualMCB
{
class VirtualMCBHandler
{
public:



/**
 * This boy needs to handle:
 * 
 * DJI motor tx handler
 * CAN rx handler
 * And then do all the extra things that MCb-Lite is supposed to do 💀
*/

public:

VirtualDJIMotorTxHandler motorTxHandler;
VirtualCANRxHandler canRxHandler;


};

}  // namespace aruwsrc::can

#endif