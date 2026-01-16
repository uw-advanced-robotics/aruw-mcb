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

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/util_macros.hpp"
#include "modm/math/filter/pid.hpp"
#include "modm/math/interpolation/linear.hpp"

namespace aruwsrc::control::launcher
{
#if defined(TARGET_HERO_ZERO)
static constexpr size_t LAUNCH_SPEED_AVERAGING_DEQUE_SIZE = 3;
#else
static constexpr size_t LAUNCH_SPEED_AVERAGING_DEQUE_SIZE = 10;
#endif

struct FlywheelConfig
{
    tap::algorithms::SmoothPidConfig velocityPidConfig;
    float orientation;
    uint8_t stage = 0;
};

#if defined(ALL_SENTRIES)
static constexpr tap::motor::MotorId LEFT_MOTOR_ID = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId RIGHT_MOTOR_ID = tap::motor::MOTOR1;
#else
static constexpr tap::motor::MotorId LEFT_MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::motor::MotorId RIGHT_MOTOR_ID = tap::motor::MOTOR2;
#endif

#ifndef TARGET_SENTRY_ECLIPSE
static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;
#endif

/** speed of ramp when you set a new desired ramp speed [rpm / ms] */
static constexpr float FRICTION_WHEEL_RAMP_SPEED = 3.0f;

#if defined(TARGET_STANDARD_VOID)
static constexpr float LAUNCHER_PID_KP = 30.0f;
static constexpr float LAUNCHER_PID_KI = 150.0f;
static constexpr float LAUNCHER_PID_KD = 0.0f;
static constexpr float LAUNCHER_PID_MAX_ERROR_SUM = 4'000.0f;
static constexpr float LAUNCHER_PID_MAX_OUTPUT = tap::motor::DjiMotor::MAX_OUTPUT_820R;
#elif defined(TARGET_SENTRY_ECLIPSE)
static constexpr float LAUNCHER_PID_KP = 30.0f;
static constexpr float LAUNCHER_PID_KI = 200.0f;
static constexpr float LAUNCHER_PID_KD = 0.0f;
static constexpr float LAUNCHER_PID_MAX_ERROR_SUM = 4'000.0f;
static constexpr float LAUNCHER_PID_MAX_OUTPUT = tap::motor::DjiMotor::MAX_OUTPUT_C610;
#else
static constexpr float LAUNCHER_PID_KP = 20.0f;
static constexpr float LAUNCHER_PID_KI = 100.0f;
static constexpr float LAUNCHER_PID_KD = 0.0f;
static constexpr float LAUNCHER_PID_MAX_ERROR_SUM = 5'000.0f;
static constexpr float LAUNCHER_PID_MAX_OUTPUT = 16'000.0f;
#endif

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
/**
 * Lookup table that maps launch speed to flywheel speed. In between points in the lookup table,
 * linear interpolation is used.
 */
#if defined(TARGET_HERO_ZERO)
static constexpr modm::Pair<float, float> LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[] = {
    {0.0f, 0.0f},
    {4.0f, 1900.0f},
    {10.0f, 3850.0f},
    {15.0f, 5750.0f},
    {16.0f, 6500.0f},
    {18.0f, 8500.0f},
};
#elif defined(TARGET_STANDARD_NULL)
static constexpr modm::Pair<float, float> LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[] = {
    {0.0f, 0.0f},
    {10.0f, 3750.0f},
    {23.0f, 6300.0f},
    {30.0f, 7000.0f},
    {32.0f, 7900.0f},
};
#elif defined(TARGET_STANDARD_VOID)
static constexpr modm::Pair<float, float> LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[] = {
    {0.0f, 0.0f},      {2.66f, 1000.0f},  {2.93f, 1100.0f},  {3.01f, 1200.0f},  {3.55f, 1300.0f},
    {3.85f, 1400.0f},  {4.00f, 1500.0f},  {4.43f, 1600.0f},  {4.77f, 1700.0f},  {4.99f, 1800.0f},
    {5.22f, 1900.0f},  {5.56f, 2000.0f},  {5.82f, 2100.0f},  {5.99f, 2200.0f},  {6.412f, 2300.0f},
    {6.75f, 2400.0f},  {7.12f, 2500.0f},  {7.36f, 2600.0f},  {7.64f, 2700.0f},  {7.88f, 2800.0f},
    {8.18f, 2900.0f},  {8.48f, 3000.0f},  {8.59f, 3100.0f},  {8.79f, 3200.0f},  {9.11f, 3300.0f},
    {9.43f, 3400.0f},  {9.87f, 3500.0f},  {10.20f, 3600.0f}, {10.27f, 3700.0f}, {11.73f, 3800.0f},
    {12.08f, 4000.0f}, {12.35f, 4100.0f}, {19.86f, 5200.0f}, {20.01f, 5300.0f}, {20.05f, 5500.0f},
    {20.62f, 5600.0f}, {21.49f, 5700.0f}, {21.74f, 5800.0f}, {21.78f, 5900.0f}, {21.93f, 6000.0f},
    {22.48f, 6100.0f}, {22.51f, 6200.0f}, {22.86f, 6300.0f}, {22.89f, 6400.0f}, {23.49f, 6500.0f},
    {23.89f, 6600.0f}, {24.12f, 6700.0f}, {24.93f, 6800.0f}, {25.17f, 6900.0f}, {26.03f, 7000.0f},
    {26.28f, 7200.0f}, {26.38f, 7300.0f}, {27.15f, 7400.0f}, {27.48f, 7500.0f}, {27.50f, 7600.0f},
    {28.19f, 7800.0f}, {29.40f, 7900.0f}};
#elif defined(TARGET_SENTRY_ECLIPSE)
static constexpr modm::Pair<float, float> LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[] = {
    {0.0f, 0.0f},      {11.33f, 4500.0f}, {11.34f, 4600.0f}, {12.3f, 4700.0f},  {13.39f, 4900.0f},
    {14.32f, 5000.0f}, {14.78f, 5100.0f}, {15.91f, 5350.0f}, {16.4f, 5500.0f},  {18.28f, 5600.0f},
    {19.18f, 5700.0f}, {19.58f, 5900.0f}, {20.21f, 6100.0f}, {20.76f, 6200.0f}, {20.88f, 6600.0f},
    {21.52f, 6700.0f}, {22.45f, 6900.0f}, {22.91f, 7000.0f}, {24.14f, 7100.0f}, {24.16f, 7200.0f},
    {24.61f, 7300.0f},

};
#else  // TARGET_DRONE, TARGET_ENGINEER
static constexpr modm::Pair<float, float> LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[] = {
    {0.0f, 0.0f},
    {15.0f, 4400.0f},
    {18.0f, 4850.0f},
    {30.0f, 7100.0f},
    {32.0f, 8400.0f},
};
#endif

#if defined(ALL_STANDARDS)
static constexpr uint32_t AGITATOR_TYPICAL_DELAY_MICROSECONDS = 90'000;
#elif defined(TARGET_HERO_ZERO)
static constexpr uint32_t AGITATOR_TYPICAL_DELAY_MICROSECONDS = 120'000;
#elif defined(TARGET_SENTRY_ECLIPSE)
static constexpr uint32_t AGITATOR_TYPICAL_DELAY_MICROSECONDS = 90'000;
#endif

#if defined(TARGET_HERO_ZERO)
static constexpr float LAUNCHER_SPEED =
    tap::communication::serial::RefSerialData::Rx::MAX_LAUNCH_SPEED_42MM - 1;
#else
static constexpr float LAUNCHER_SPEED =
    tap::communication::serial::RefSerialData::Rx::MAX_LAUNCH_SPEED_17MM - 3;
#endif

}  // namespace aruwsrc::control::launcher

#endif  // LAUNCHER_CONSTANTS_HPP_
