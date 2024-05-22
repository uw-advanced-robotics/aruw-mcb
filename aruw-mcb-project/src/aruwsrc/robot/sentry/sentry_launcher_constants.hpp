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

#ifndef SENTRY_LAUNCHER_CONSTANTS_HPP_
#define SENTRY_LAUNCHER_CONSTANTS_HPP_

#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/util_macros.hpp"
#include "modm/math/filter/pid.hpp"
#include "modm/math/interpolation/linear.hpp"

// @todo: violates namespace convention but necessary to prevent pollution
namespace aruwsrc::robot::sentry::launcher
{
static constexpr size_t LAUNCH_SPEED_AVERAGING_DEQUE_SIZE = 10;

// @todo this is not the greatest
static constexpr tap::motor::MotorId LEFT_MOTOR_ID_TURRETLEFT = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId RIGHT_MOTOR_ID_TURRETLEFT = tap::motor::MOTOR1;
static constexpr tap::motor::MotorId LEFT_MOTOR_ID_TURRETRIGHT = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId RIGHT_MOTOR_ID_TURRETRIGHT = tap::motor::MOTOR1;

/** speed of ramp when you set a new desired ramp speed [rpm / ms] */
static constexpr float FRICTION_WHEEL_RAMP_SPEED = 3.0f;

static constexpr float LAUNCHER_PID_KP = 20.0f;
static constexpr float LAUNCHER_PID_KI = 0.2f;
static constexpr float LAUNCHER_PID_KD = 0.0f;
static constexpr float LAUNCHER_PID_MAX_ERROR_SUM = 5'000.0f;
static constexpr float LAUNCHER_PID_MAX_OUTPUT = 16'000.0f;

/**
 * Lookup table that maps launch speed to flywheel speed. In between points in the lookup table,
 * linear interpolation is used.
 */
static constexpr modm::Pair<float, float> LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[] = {
    {0.0f, 0.0f},
    {15.0f, 4400.0f},
    {18.0f, 4850.0f},
    {30.0f, 6900.0f},
    {32.0f, 8400.0f},
};

// Desired speed of projectile in m/s
static constexpr float DESIRED_LAUNCH_SPEED = 5.0f;

static constexpr uint32_t AGITATOR_TYPICAL_DELAY_MICROSECONDS = 60'000;

}  // namespace aruwsrc::robot::sentry::launcher

#endif  // SENTRY_LAUNCHER_CONSTANTS_HPP_
