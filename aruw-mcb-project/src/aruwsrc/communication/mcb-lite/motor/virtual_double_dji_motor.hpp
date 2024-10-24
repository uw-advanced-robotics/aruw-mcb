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

#ifndef VIRTUAL_DOUBLE_DJI_MOTOR_HPP_
#define VIRTUAL_DOUBLE_DJI_MOTOR_HPP_

#include "tap/drivers.hpp"
#include "tap/motor/double_dji_motor.hpp"

#include "../mcb_lite.hpp"

using namespace tap::motor;

namespace aruwsrc::virtualMCB
{
/**
 * This class builds off of DoubleDjiMotor, but changes motor communication to talk to a virtual MCB
 */
class VirtualDoubleDjiMotor : public tap::motor::DoubleDjiMotor
{
public:
    VirtualDoubleDjiMotor(
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
        int64_t encRevolutions = 0);

    void initialize() override;

    void attachSelfToRxHandler();

private:
    MCBLite* mcbLite;
};

}  // namespace aruwsrc::virtualMCB

#endif
