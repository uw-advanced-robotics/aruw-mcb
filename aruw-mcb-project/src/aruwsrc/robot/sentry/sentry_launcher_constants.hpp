/*
 * Copyright (c) 2022-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include <cstddef>
#include <cstdint>

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/launcher/friction_wheel_interface.hpp"
#include "modm/math/interpolation/linear.hpp"

// Do not include this file directly: use launcher_constants.hpp instead.
#ifndef LAUNCHER_CONSTANTS_HPP_
#error "Do not include this file directly! Use launcher_constants.hpp instead."
#endif

#if !defined(TARGET_SENTRY_ACHLYS)
#error "Attempted to include sentry_launcher_constants.hpp for non-sentry target."
#endif

namespace aruwsrc::control::launcher
{
static constexpr size_t LAUNCH_SPEED_AVERAGING_DEQUE_SIZE = 10;

static constexpr tap::motor::MotorId LEFT_MOTOR_ID = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId RIGHT_MOTOR_ID = tap::motor::MOTOR1;

/** speed of ramp when you set a new desired ramp speed [rpm / ms] */
static constexpr float FRICTION_WHEEL_RAMP_SPEED = 3.0f;

static constexpr float LAUNCHER_PID_KP = 14.0106f;
static constexpr float LAUNCHER_PID_KI = 31.6228f;
static constexpr float LAUNCHER_PID_KD = 0.0f;
static constexpr float LAUNCHER_PID_MAX_ERROR_SUM = 4'000.0f;
static constexpr float LAUNCHER_PID_MAX_OUTPUT = tap::motor::DjiMotor::MAX_OUTPUT_C610;

static constexpr tap::algorithms::SmoothPidConfig VELOCITY_PID_CONFIG(
    LAUNCHER_PID_KP,
    LAUNCHER_PID_KI,
    LAUNCHER_PID_KD,
    LAUNCHER_PID_MAX_ERROR_SUM,
    LAUNCHER_PID_MAX_OUTPUT);
static constexpr FlywheelConfig WHEEL_CONFIG = {VELOCITY_PID_CONFIG, 90.0f};

static constexpr float LAUNCHER_SPEED_CORRECTION_PID_KP = 0.0f;
static constexpr float LAUNCHER_SPEED_CORRECTION_PID_KI = 0.749f;
static constexpr float LAUNCHER_SPEED_CORRECTION_PID_KD = 0.0f;
static constexpr float LAUNCHER_SPEED_CORRECTION_PID_MAX_ERROR_SUM = 500.0f;
static constexpr float LAUNCHER_SPEED_CORRECTION_PID_MAX_OUTPUT = 750.0f;
static constexpr tap::algorithms::SmoothPidConfig LAUNCHER_SPEED_CORRECTION_PID_CONFIG = {
    LAUNCHER_SPEED_CORRECTION_PID_KP,
    LAUNCHER_SPEED_CORRECTION_PID_KI,
    LAUNCHER_SPEED_CORRECTION_PID_KD,
    LAUNCHER_SPEED_CORRECTION_PID_MAX_ERROR_SUM,
    LAUNCHER_SPEED_CORRECTION_PID_MAX_OUTPUT};

static constexpr modm::Pair<float, float> LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[] = {
    {0.0f, 0.0f},      {11.33f, 4500.0f}, {11.34f, 4600.0f}, {12.3f, 4700.0f},  {13.39f, 4900.0f},
    {14.32f, 5000.0f}, {14.78f, 5100.0f}, {15.91f, 5350.0f}, {16.4f, 5500.0f},  {18.28f, 5600.0f},
    {19.18f, 5700.0f}, {19.58f, 5900.0f}, {20.21f, 6100.0f}, {20.76f, 6200.0f}, {20.88f, 6600.0f},
    {21.52f, 6700.0f}, {22.45f, 6900.0f}, {22.91f, 7000.0f}, {24.14f, 7100.0f}, {24.16f, 7200.0f},
    {24.61f, 7300.0f},
};

static constexpr uint32_t AGITATOR_TYPICAL_DELAY_MICROSECONDS = 90'000;
static constexpr float LAUNCHER_SPEED =
    tap::communication::serial::RefSerialData::Rx::MAX_LAUNCH_SPEED_17MM - 3;

}  // namespace aruwsrc::control::launcher

#endif  // SENTRY_LAUNCHER_CONSTANTS_HPP_
