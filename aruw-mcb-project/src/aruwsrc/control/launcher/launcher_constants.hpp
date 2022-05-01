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

#ifndef LAUNCHER_CONSTANTS_HPP_
#define LAUNCHER_CONSTANTS_HPP_

#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/util_macros.hpp"
#include "modm/math/filter/pid.hpp"
#include "modm/math/interpolation/linear.hpp"

namespace aruwsrc::control::launcher
{
#if defined(TARGET_HERO) || defined(ALL_SENTINELS)
static constexpr tap::motor::MotorId LEFT_MOTOR_ID = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId RIGHT_MOTOR_ID = tap::motor::MOTOR1;
#else
static constexpr tap::motor::MotorId LEFT_MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::motor::MotorId RIGHT_MOTOR_ID = tap::motor::MOTOR2;
#endif

#if defined(TARGET_SENTINEL_2022)
static constexpr tap::can::CanBus TURRET1_CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;
static constexpr tap::can::CanBus TURRET2_CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS2;
#else
static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;
#endif

/** speed of ramp when you set a new desired ramp speed [rpm / ms] */
static constexpr float FRICTION_WHEEL_RAMP_SPEED = 1.0f;

static constexpr float LAUNCHER_PID_KP = 0.0f;
static constexpr float LAUNCHER_PID_KI = 0.0f;
static constexpr float LAUNCHER_PID_KD = 0.0f;
static constexpr float LAUNCHER_PID_MAX_ERROR_SUM = 0.0f;
static constexpr float LAUNCHER_PID_MAX_OUTPUT = 0.0f;

/**
 * Lookup table that maps launch speed to flywheel speed. In between points in the lookup table,
 * linear interpolation is used.
 */
#ifdef TARGET_HERO
static constexpr modm::Pair<float, float> LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[] = {
    {0.0f, 0.0f},
    {10, 1000.0f},
    {16.0f, 1000.0f},
    {20.0f, 1000.0f},
};
#else
static constexpr modm::Pair<float, float> LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[] = {
    {0.0f, 0.0f},
    {15.0f, 1000.0f},
    {18.0f, 1000.0f},
    {30.0f, 1000.0f},
    {32.0f, 1000.0f},
};
#endif
}  // namespace aruwsrc::control::launcher

#endif  // LAUNCHER_CONSTANTS_HPP_
