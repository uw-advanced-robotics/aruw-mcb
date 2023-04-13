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

#ifndef VIRTUAL_DJI_MOTOR_HPP_
#define VIRTUAL_DJI_MOTOR_HPP_

#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"

#include "virtual_can_rx_handler.hpp"
#include "virtual_dji_motor_tx_handler.hpp"

using namespace tap::motor;

namespace aruwsrc::virtualMCB
{
class VirtualDjiMotor : public tap::motor::DjiMotor
{
public:
    VirtualDjiMotor(
        tap::Drivers* drivers,
        MotorId desMotorIdentifier,
        tap::can::CanBus motorCanBus,
        VirtualDJIMotorTxHandler* txMotorHandler,
        VirtualCANRxHandler* rxHandler,
        bool isInverted,
        const char* name,
        uint16_t encoderWrapped = DjiMotor::ENC_RESOLUTION / 2,
        int64_t encoderRevolutions = 0);

    void initialize() override;

    void attachSelfToRxHandler();

private:
    VirtualDJIMotorTxHandler* txMotorHandler;
    VirtualCANRxHandler* rxHandler;
};

}  // namespace aruwsrc::virtualMCB

#endif