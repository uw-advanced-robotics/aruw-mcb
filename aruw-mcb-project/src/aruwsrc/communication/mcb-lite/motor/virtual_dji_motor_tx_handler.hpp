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

#ifndef VIRTUAL_DJI_MOTOR_TX_HANDLER_HPP_
#define VIRTUAL_DJI_MOTOR_TX_HANDLER_HPP_

#include "tap/drivers.hpp"
#include "tap/motor/dji_motor_tx_handler.hpp"

#include "../can_bus.hpp"

namespace aruwsrc::virtualMCB
{
class VirtualDJIMotorTxHandler : public tap::motor::DjiMotorTxHandler
{
public:
    VirtualDJIMotorTxHandler(tap::Drivers* drivers, CanBus canbus);

	void encodeAndSendCanData();

private:
    CanBus canbus;
};

}  // namespace aruwsrc::virtualMCB

#endif