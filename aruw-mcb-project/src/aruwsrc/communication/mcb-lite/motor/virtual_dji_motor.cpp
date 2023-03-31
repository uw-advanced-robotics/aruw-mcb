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

#include "virtual_dji_motor.hpp"

#include "tap/communication/can/can.hpp"
#include "tap/communication/can/can_bus.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"

namespace aruwsrc::virtualMCB
{

VirtualDjiMotor::VirtualDjiMotor(
    tap::Drivers* drivers,
    MotorId desMotorIdentifier,
    tap::can::CanBus motorCanBus,
    VirtualDJIMotorTxHandler* txMotorHandler,
    VirtualCANRxHandler* rxHandler,
    bool isInverted,
    const char* name,
    uint16_t encoderWrapped,
    int64_t encoderRevolutions)
    : DjiMotor(
          drivers,
          desMotorIdentifier,
          motorCanBus,
          isInverted,
          name,
          encoderWrapped,
          encoderRevolutions),
      txMotorHandler(txMotorHandler),
      rxHandler(rxHandler)
{
}

void VirtualDjiMotor::initialize()
{
    txMotorHandler->addMotorToManager(this);
    attachSelfToRxHandler();
}

void VirtualDjiMotor::attachSelfToRxHandler() { rxHandler->attachReceiveHandler(this); }

}  // namespace aruwsrc::virtualMCB