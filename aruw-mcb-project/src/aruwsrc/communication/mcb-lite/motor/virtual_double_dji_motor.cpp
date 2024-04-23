/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "virtual_double_dji_motor.hpp"

#include "tap/communication/can/can.hpp"
#include "tap/communication/can/can_bus.hpp"
#include "tap/drivers.hpp"

namespace aruwsrc::virtualMCB
{
VirtualDoubleDjiMotor::VirtualDoubleDjiMotor(
    tap::Drivers* drivers,
    MCBLite* mcbLite,
    MotorId desMotorIdentifierOne,
    MotorId desMotorIdentifierTwo,
    tap::can::CanBus motorCanBusOne,
    tap::can::CanBus motorCanBusTwo,
    bool isInvertedOne,
    bool isInvertedTwo,
    const char* nameOne,
    const char* nameTwo,
    uint16_t encWrapped = DjiMotor::ENC_RESOLUTION / 2,
    int64_t encRevolutions = 0)
    : DoubleDjiMotor(
          drivers,
          desMotorIdentifierOne,
          desMotorIdentifierTwo,
          motorCanBusOne,
          motorCanBusTwo,
          isInvertedOne,
          isInvertedTwo,
          nameOne,
          nameTwo,
          encWrapped,
          encRevolutions),
      mcbLite(mcbLite)
{
}

void VirtualDoubleDjiMotor::initialize()
{
    mcbLite->motorTxHandler.addMotorToManager(&motorOne);
    mcbLite->motorTxHandler.addMotorToManager(&motorTwo);
    attachSelfToRxHandler();
}

void VirtualDoubleDjiMotor::attachSelfToRxHandler()
{
    mcbLite->canRxHandler.attachReceiveHandler(&motorOne);
    mcbLite->canRxHandler.attachReceiveHandler(&motorTwo);
}

}  // namespace aruwsrc::virtualMCB
