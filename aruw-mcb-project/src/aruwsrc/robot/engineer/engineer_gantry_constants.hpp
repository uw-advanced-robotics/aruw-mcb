/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef ENGINEER_GANTRY_CONSTANTS_HPP_
#define ENGINEER_GANTRY_CONSTANTS_HPP_

#include "tap/motor/dji_motor.hpp"

namespace aruwsrc::engineer
{
static constexpr tap::can::CanBus GANTRY_CAN_BUS = tap::can::CanBus::CAN_BUS1;
static constexpr tap::motor::MotorId GANTRY_LIFT_LEFT_MOTOR_ID = tap::motor::MotorId::MOTOR1;
static constexpr tap::motor::MotorId GANTRY_LIFT_RIGHT_MOTOR_ID = tap::motor::MotorId::MOTOR2;
static constexpr tap::motor::MotorId GANTRY_EXTENSION_MOTOR_ID = tap::motor::MotorId::MOTOR3;
static constexpr tap::motor::MotorId WRIST_LEFT_MOTOR_ID = tap::motor::MotorId::MOTOR4;
static constexpr tap::motor::MotorId WRIST_RIGHT_MOTOR_ID = tap::motor::MotorId::MOTOR5;
static constexpr tap::motor::MotorId WRIST_ROLL_MOTOR_ID = tap::motor::MotorId::MOTOR6;
}  // namespace aruwsrc::engineer
#endif  // ENGINEER_GANTRY_CONSTANTS_HPP_