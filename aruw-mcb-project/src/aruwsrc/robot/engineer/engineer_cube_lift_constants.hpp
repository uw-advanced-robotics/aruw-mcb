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

#ifndef ENGINEER_CUBE_LIFT_CONSTANTS_HPP_
#define ENGINEER_CUBE_LIFT_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/gpio/digital.hpp"
#include "tap/motor/dji_motor.hpp"

namespace aruwsrc::engineer
{
static constexpr tap::motor::MotorId CUBE_LIFT_MOTOR_ID = tap::motor::MOTOR7;

static constexpr tap::can::CanBus LIFT_MOTOR_CAN_BUS = tap::can::CanBus::CAN_BUS2;

static constexpr tap::gpio::Digital::InputPin CUBELIFT_LIMITSWITCH_PORT =
    tap::gpio::Digital::InputPin::B;

static constexpr float MM_PER_REVOLUTION = 74.63f / M_TWOPI;

static constexpr tap::algorithms::SmoothPidConfig LIFT_MOTOR_PID_CONFIG = {
    .kp = 300.0f,
    .ki = 50.0f,
    .kd = 25.0f,
    .maxICumulative = 2000.0f,
    .maxOutput = 6000.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr float CUBE_LIFT_MOVE_SPEED = -2;

static constexpr float ONE_CUBE_SETPOINT = -40;
static constexpr float TWO_CUBE_SETPOINT = -220;
static constexpr float THREE_CUBE_SETPOINT = -310;

static constexpr float LIFT_UPPER_BOUND = THREE_CUBE_SETPOINT;

}  // namespace aruwsrc::engineer
#endif