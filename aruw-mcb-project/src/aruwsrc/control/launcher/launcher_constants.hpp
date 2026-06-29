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

#include "friction_wheel_interface.hpp"

namespace aruwsrc::control::launcher
{
#if defined(TARGET_HERO_NEPTUNE)
static constexpr size_t LAUNCH_SPEED_AVERAGING_DEQUE_SIZE = 3;
#else
static constexpr size_t LAUNCH_SPEED_AVERAGING_DEQUE_SIZE = 10;
#endif

#if defined(TARGET_FLYWHEEL_TESTING)
struct FlywheelRpms
{
    float leftRpm;
    float rightRpm;
    float lowerRpm;
    float upperRpm;
};

static constexpr FlywheelRpms flywheelTestingRpms{
    .leftRpm = 6000.0f,
    .rightRpm = 6000.0f,
    .lowerRpm = 6000.0f,
    .upperRpm = 6000.0f};
#endif

#if defined(ALL_SENTRIES)
static constexpr tap::motor::MotorId LEFT_MOTOR_ID = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId RIGHT_MOTOR_ID = tap::motor::MOTOR1;
#elif defined(TARGET_FLYWHEEL_TESTING)
static constexpr tap::motor::MotorId UPPER_MOTOR_ID = tap::motor::MOTOR6;
static constexpr tap::motor::MotorId LOWER_MOTOR_ID = tap::motor::MOTOR4;
static constexpr tap::motor::MotorId LEFT_MOTOR_ID = tap::motor::MOTOR3;
static constexpr tap::motor::MotorId RIGHT_MOTOR_ID = tap::motor::MOTOR1;
#elif defined(TARGET_HERO_NEPTUNE)
static constexpr tap::motor::MotorId LEFT_FRONT_MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::motor::MotorId RIGHT_FRONT_MOTOR_ID = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId LEFT_BACK_MOTOR_ID = tap::motor::MOTOR3;
static constexpr tap::motor::MotorId RIGHT_BACK_MOTOR_ID = tap::motor::MOTOR4;
#else
static constexpr tap::motor::MotorId LEFT_MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::motor::MotorId RIGHT_MOTOR_ID = tap::motor::MOTOR2;
#endif

#if !defined(ALL_SENTRIES)
static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;
#endif

/** speed of ramp when you set a new desired ramp speed [rpm / ms] */
static constexpr float FRICTION_WHEEL_RAMP_SPEED = 3.0f;

#if defined(TARGET_SENTRY_ACHLYS)
static constexpr float LAUNCHER_PID_KP = 14.0106f;
static constexpr float LAUNCHER_PID_KI = 31.6228f;
static constexpr float LAUNCHER_PID_KD = 0.0f;
static constexpr float LAUNCHER_PID_MAX_ERROR_SUM = 4'000.0f;
static constexpr float LAUNCHER_PID_MAX_OUTPUT = tap::motor::DjiMotor::MAX_OUTPUT_C610;
#elif defined(TARGET_FLYWHEEL_TESTING)
static constexpr float LAUNCHER_PID_KP = 20.0f;
static constexpr float LAUNCHER_PID_KI = 100.0f;
static constexpr float LAUNCHER_PID_KD = 0.0f;
static constexpr float LAUNCHER_PID_MAX_ERROR_SUM = 5'000.0f;
static constexpr float LAUNCHER_PID_MAX_OUTPUT = tap::motor::DjiMotor::MAX_OUTPUT_820R;
static constexpr tap::algorithms::SmoothPidConfig LEFT_VELOCITY_PID_CONFIG(
    LAUNCHER_PID_KP,
    LAUNCHER_PID_KI,
    LAUNCHER_PID_KD,
    LAUNCHER_PID_MAX_ERROR_SUM,
    tap::motor::DjiMotor::MAX_OUTPUT_C620);
static constexpr FlywheelConfig LEFT_WHEEL_CONFIG = {LEFT_VELOCITY_PID_CONFIG, 270.0f};
#else
static constexpr float LAUNCHER_PID_KP = 14.0106f;
static constexpr float LAUNCHER_PID_KI = 31.6228f;
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
#if defined(TARGET_HERO_NEPTUNE)
static constexpr modm::Pair<float, float> LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[] = {
    {0.0f, 0.0f},
    {5.33961248f, 2000.0f},
    {7.2889533f, 2500.0f},
    {9.09577465f, 3000.0f},
    {10.8697052f, 3500.0f},
    {12.4806519f, 4000.0f},
    {13.8437967f, 4500.0f},
    {15.3708878f, 5000.0f},
    {16.868948f, 5500.0f},
    {18.4827785f, 6000.0f},
    {19.7700138f, 6500.0f},
    {20.3329659f, 7000.0f},
    {21.3121357f, 7500.0f},
    {23.184721f, 8000.0f},
};
#elif defined(TARGET_STANDARD_NULL)
static constexpr modm::Pair<float, float> LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[] = {
    {0.0f, 0.0f},
    {10.0f, 3750.0f},
    {23.0f, 6300.0f},
    {30.0f, 7000.0f},
    {32.0f, 7900.0f},
};
#elif defined(TARGET_STANDARD_PHOBOS)
static constexpr modm::Pair<float, float> LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[] = {
    {0.0f, 0.0f},
    {8.39017773f, 3000.0f},
    {10.4645939f, 3250.0f},
    {10.9344473f, 3500.0f},
    {13.401207f, 3750.0f},
    {14.6625366f, 4000.0f},
    {16.0350933f, 4250.0f},
    {17.2190266f, 4500.0f},
    {18.1780758f, 4750.0f},
    {19.3804283f, 5000.0f},
    {20.3881416f, 5250.0f},
    {20.9700661f, 5500.0f},
    {22.0063496f, 5750.0f},
    {22.8945255f, 6000.0f},
    {23.5541458f, 6250.0f},
    {24.2752342f, 6500.0f},
    {24.9456329f, 6750.0f},
    {25.8669395f, 7250.0f},
    {26.2437325f, 7750.0f}};
#elif defined(TARGET_STANDARD_DEIMOS)
static constexpr modm::Pair<float, float> LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[] = {
    {0.0f, 0.0f},
    {8.39017773f, 3000.0f},
    {10.4645939f, 3250.0f},
    {10.9344473f, 3500.0f},
    {13.401207f, 3750.0f},
    {14.6625366f, 4000.0f},
    {16.0350933f, 4250.0f},
    {17.2190266f, 4500.0f},
    {18.1780758f, 4750.0f},
    {19.3804283f, 5000.0f},
    {20.3881416f, 5250.0f},
    {20.9700661f, 5500.0f},
    {22.0063496f, 5750.0f},
    {22.8945255f, 6000.0f},
    {23.5541458f, 6250.0f},
    {24.2752342f, 6500.0f},
    {24.9456329f, 6750.0f},
    {25.8669395f, 7250.0f},
    {26.2437325f, 7750.0f}};
#elif defined(TARGET_SENTRY_ACHLYS)
static constexpr modm::Pair<float, float> LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[] = {
    {0.0f, 0.0f},
    {14.92f, 4500.0f},
    {16.22f, 4750.0f},
    {17.39f, 5000.0f},
    {18.93f, 5250.0f},
    {19.47f, 5500.0f},
    {21.07f, 5750.0f},
    {22.19f, 6000.0f},
    {23.31f, 6250.0f},
    {24.39f, 6500.0f},
    {25.17f, 6750.0f},
    {26.06f, 7000.0f},
    {27.10f, 7250.0f},
    {27.67f, 7500.0f},
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
#elif defined(TARGET_HERO_NEPTUNE)
static constexpr uint32_t AGITATOR_TYPICAL_DELAY_MICROSECONDS = 120'000;
#elif defined(TARGET_SENTRY_ACHLYS)
static constexpr uint32_t AGITATOR_TYPICAL_DELAY_MICROSECONDS = 90'000;
#endif

#if defined(TARGET_HERO_NEPTUNE)
static constexpr float LAUNCHER_SPEED = 14.5f;
#else
static constexpr float LAUNCHER_SPEED =
    tap::communication::serial::RefSerialData::Rx::MAX_LAUNCH_SPEED_17MM - 3;
#endif

}  // namespace aruwsrc::control::launcher

#endif  // LAUNCHER_CONSTANTS_HPP_
