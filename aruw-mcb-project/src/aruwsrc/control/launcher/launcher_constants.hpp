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

#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/util_macros.hpp"
#include "modm/math/filter/pid.hpp"
#include "modm/math/interpolation/linear.hpp"

namespace aruwsrc::control::launcher
{
#if defined(TARGET_HERO_PERSEUS)
static constexpr size_t LAUNCH_SPEED_AVERAGING_DEQUE_SIZE = 3;
#else
static constexpr size_t LAUNCH_SPEED_AVERAGING_DEQUE_SIZE = 10;
#endif

#if defined(TARGET_HERO_PERSEUS) || defined(ALL_SENTRIES)
static constexpr tap::motor::MotorId LEFT_MOTOR_ID = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId RIGHT_MOTOR_ID = tap::motor::MOTOR1;
#else
static constexpr tap::motor::MotorId LEFT_MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::motor::MotorId RIGHT_MOTOR_ID = tap::motor::MOTOR2;
#endif

#ifndef TARGET_SENTRY_HYDRA
static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;
#endif

/** speed of ramp when you set a new desired ramp speed [rpm / ms] */
static constexpr float FRICTION_WHEEL_RAMP_SPEED = 3.0f;

#if defined(TARGET_STANDARD_ORION) || defined(TARGET_STANDARD_CYGNUS)
static constexpr float LAUNCHER_PID_KP = 30.0f;
static constexpr float LAUNCHER_PID_KI = 0.3f;
static constexpr float LAUNCHER_PID_KD = 0.0f;
static constexpr float LAUNCHER_PID_MAX_ERROR_SUM = 4'000.0f;
static constexpr float LAUNCHER_PID_MAX_OUTPUT = tap::motor::DjiMotor::MAX_OUTPUT_GM3510;
#elif defined(TARGET_SENTRY_HYDRA)
static constexpr float LAUNCHER_PID_KP = 30.0f;
static constexpr float LAUNCHER_PID_KI = 0.4f;
static constexpr float LAUNCHER_PID_KD = 0.0f;
static constexpr float LAUNCHER_PID_MAX_ERROR_SUM = 4'000.0f;
static constexpr float LAUNCHER_PID_MAX_OUTPUT = tap::motor::DjiMotor::MAX_OUTPUT_C610;
#else
static constexpr float LAUNCHER_PID_KP = 20.0f;
static constexpr float LAUNCHER_PID_KI = 0.2f;
static constexpr float LAUNCHER_PID_KD = 0.0f;
static constexpr float LAUNCHER_PID_MAX_ERROR_SUM = 5'000.0f;
static constexpr float LAUNCHER_PID_MAX_OUTPUT = 16'000.0f;
#endif

/**
 * Lookup table that maps launch speed to flywheel speed. In between points in the lookup table,
 * linear interpolation is used.
 */
#if defined(TARGET_HERO_PERSEUS)
static constexpr modm::Pair<float, float> LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[] = {
    {0.0f, 0.0f},
    {4.0f, 1900.0f},
    {10.0f, 3850.0f},
    {15.0f, 5750.0f},
    {16.0f, 6350.0f},
    {18.0f, 8500.0f},
};
#elif defined(TARGET_STANDARD_ORION)
static constexpr modm::Pair<float, float> LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[] = {
    {0.0f, 0.0f},
    {10.0f, 3750.0f},
    {24.0f, 6200.0f},
    {30.0f, 7000.0f},
    {32.0f, 7900.0f},
};
#elif defined(TARGET_STANDARD_CYGNUS)
static constexpr modm::Pair<float, float> LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[] = {
    {0.0f, 0.0f},
    {10.0f, 3750.0f},
    {24.0f, 6200.0f},
    {30.0f, 7300.0f},
    {32.0f, 8000.0f},
};
#elif defined(TARGET_STANDARD_SPIDER)
static constexpr modm::Pair<float, float> LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[] = {
    {0.0f, 0.0f},
    {15.0f, 4325.0f},
    {18.0f, 4800.0f},
    {30.0f, 6900.0f},
    {32.0f, 8400.0f},
};
#elif defined(TARGET_SENTRY_HYDRA)
static constexpr modm::Pair<float, float> LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[] = {
    {0.0f, 0.0f},
    {15.0f, 4900.0f},
    {18.0f, 6050.0f},
    {27.5f, 7000.0f},
    {30.0f, 7700.0f},
    {32.0f, 8400.0f},
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
static constexpr uint32_t AGITATOR_TYPICAL_DELAY_MICROSECONDS = 80'000;
#elif defined(TARGET_HERO_PERSEUS)
static constexpr uint32_t AGITATOR_TYPICAL_DELAY_MICROSECONDS = 120'000;
#elif defined(TARGET_SENTRY_HYDRA)
static constexpr uint32_t AGITATOR_TYPICAL_DELAY_MICROSECONDS = 90'000;
#endif

#if defined(TARGET_HERO_PERSEUS)
static constexpr float LAUNCHER_SPEED =
    tap::communication::serial::RefSerialData::Rx::MAX_LAUNCH_SPEED_42MM - 1;
#else
static constexpr float LAUNCHER_SPEED =
    tap::communication::serial::RefSerialData::Rx::MAX_LAUNCH_SPEED_17MM - 1;
#endif

}  // namespace aruwsrc::control::launcher

#endif  // LAUNCHER_CONSTANTS_HPP_
