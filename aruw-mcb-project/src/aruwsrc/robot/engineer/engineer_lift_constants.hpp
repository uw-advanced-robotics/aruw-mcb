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

#ifndef ENGINEER_LIFT_CONSTANTS_HPP_
#define ENGINEER_LIFT_CONSTANTS_HPP_

#include "tap/communication/gpio/digital.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/algorithms/smooth_pid.hpp"

namespace aruwsrc::robot::engineer
{
static constexpr tap::motor::MotorId CUBE_LIFT_MOTOR_ID =
    tap::motor::MOTOR3;  // TODO: UPDATE W CORRECT VALUE

static constexpr tap::can::CanBus LIFT_MOTOR_CAN_BUS =
    tap::can::CanBus::CAN_BUS1;  // TODO: UPDATE W CORRECT VALUE

static constexpr tap::gpio::Digital::InputPin LIMITSWITCH_PORT =
    tap::gpio::Digital::InputPin::D;  // TODO: UPDATE W CORRECT VALUE

static constexpr int16_t FEEDFORWARD = 1000;  // TODO: UPDATE W CORRECT VALUE

static constexpr tap::algorithms::SmoothPidConfig LIFT_MOTOR_PID_CONFIG = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

float MANUAL_MOVE_SPEED = 10; //TODO: choose value alter

float ONE_CUBE_SETPOINT = 100;
float TWO_CUBE_SETPOINT = 200;
float THREE_CUBE_SETPOINT = 300; //TODO: update correct values

}  // namespace aruwsrc::robot::engineer
#endif