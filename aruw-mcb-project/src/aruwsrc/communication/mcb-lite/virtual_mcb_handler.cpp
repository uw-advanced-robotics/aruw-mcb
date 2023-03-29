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

#include "virtual_mcb_handler.hpp"

#include "tap/drivers.hpp"

namespace aruwsrc::virtualMCB
{

VirtualMCBHandler::VirtualMCBHandler(
    tap::Drivers* drivers,
    tap::communication::serial::Uart::UartPort port)
    : canbus(&VirtualCanBus(drivers, port)),
      motorTxHandler(&VirtualDJIMotorTxHandler(drivers, canbus)),
      canRxHandler(&VirtualCANRxHandler(drivers, canbus))
{
}

void VirtualMCBHandler::refresh(){
	canRxHandler->pollCanData();
	motorTxHandler->encodeAndSendCanData();
}

VirtualCanBus* VirtualMCBHandler::getCanbus() { return canbus; }
VirtualCANRxHandler* VirtualMCBHandler::getRxHandler() { return canRxHandler; }
VirtualDJIMotorTxHandler* VirtualMCBHandler::getDjiMotorHandler() { return motorTxHandler; }

}  // namespace aruwsrc::virtualMCB