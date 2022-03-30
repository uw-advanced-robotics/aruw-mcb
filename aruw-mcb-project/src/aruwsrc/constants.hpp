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

#ifndef CONSTANTS_HPP_
#define CONSTANTS_HPP_

#include "aruwsrc/util_macros.hpp"

#if defined(ALL_SOLDIERS)
#include "soldier_constants.hpp"
#elif defined(TARGET_HERO)
#include "hero_constants.hpp"
#elif defined(TARGET_SENTINEL)
#include "sentinel_constants.hpp"
#elif defined(TARGET_DRONE)
#include "drone_constants.hpp"
#elif defined(TARGET_ENGINEER)
#include "engineer_constants.hpp"
#endif

#include "tap/motor/dji_motor.hpp"

namespace aruwsrc::chassis
{
// hardware constants, not specific to any particular chassis
static constexpr tap::motor::MotorId LEFT_FRONT_MOTOR_ID = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId LEFT_BACK_MOTOR_ID = tap::motor::MOTOR3;
static constexpr tap::motor::MotorId RIGHT_FRONT_MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::motor::MotorId RIGHT_BACK_MOTOR_ID = tap::motor::MOTOR4;

static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS2;
}  // namespace aruwsrc::chassis

namespace aruwsrc::control::launcher
{
    static constexpr tap::motor::MotorId LEFT_MOTOR_ID = tap::motor::MOTOR2;
    static constexpr tap::motor::MotorId RIGHT_MOTOR_ID = tap::motor::MOTOR1;
    static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

    /** speed of ramp when you set a new desired ramp speed [rpm / ms] */
    static constexpr float FRICTION_WHEEL_RAMP_SPEED = 1.0f;

    // TODO: Unfuck this
    static constexpr float PID_P = 20.0f;
    static constexpr float PID_I = 0.2f;
    static constexpr float PID_D = 0.0f;
    static constexpr float PID_MAX_ERROR_SUM = 5'000.0f;
    static constexpr float PID_MAX_OUTPUT = 16000.0f;
}  // namespace aruwsrc::control::launcher

#endif  // TURRET_CONTROLLER_CONSTANTS_HPP_
