/*
* Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/motor/dji_motor.hpp"

#include "tap/communication/gpio/digital.hpp"

namespace aruwsrc::robot::engineer
{
static constexpr tap::motor::MotorId CUBE_LIFT_MOTOR_ID = tap::motor::MOTOR1;  //TODO: UPDATE W CORRECT VALUE
 

static constexpr tap::can::CanBus LIFT_MOTOR_CAN_BUS = tap::can::CanBus::CAN_BUS2; // TODO: UPDATE W CORRECT VALUE

static constexpr tap::gpio::Digital::InputPin LIMITSWITCH_PORT = tap::gpio::Digital::InputPin::D; //TODO: UPDATE W CORRECT VALUE

}
