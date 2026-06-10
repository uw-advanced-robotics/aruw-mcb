/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef STANDARD_CHASSIS_CONSTANTS_HPP_
#define STANDARD_CHASSIS_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/gpio/analog.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/chassis/beyblade_config.hpp"
#include "modm/math/interpolation/linear.hpp"

// Do not include this file directly: use chassis_constants.hpp instead.
#ifndef CHASSIS_CONSTANTS_HPP_
#error "Do not include this file directly! Use chassis_constants.hpp instead."
#endif

namespace aruwsrc::control::chassis
{
static constexpr float CAP_BANK_CAPACITANCE = 4.358f;
/**
 * The minimum desired wheel speed for chassis rotation when translational scaling via
 * calculateRotationTranslationalGain is performed.
 */
static constexpr float MIN_ROTATION_THRESHOLD = 80.0f;
#if defined(TARGET_STANDARD_NULL)
inline constexpr float CHASSIS_GEARBOX_RATIO = tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508;
#else
// Custom gearbox ratio
inline constexpr float CHASSIS_GEARBOX_RATIO = (17.0f / 268.0f);
#endif
/**
 * Pin to use for current sensing
 */
static constexpr tap::gpio::Analog::Pin CURRENT_SENSOR_PIN = tap::gpio::Analog::Pin::S;

/// @see power_limiter.hpp for what these mean
static constexpr float STARTING_ENERGY_BUFFER = 60.0f;
static constexpr float ENERGY_BUFFER_LIMIT_THRESHOLD = 60.0f;
static constexpr float ENERGY_BUFFER_CRIT_THRESHOLD = 10.0f;

static constexpr float VELOCITY_PID_KV = 0.07f;
static constexpr float VELOCITY_PID_KS = 1.0f;
/**
 * Rotation PD: A PD controller for chassis autorotation, which causes the chassis to follow the
 * turret's pointing direction
 */
static constexpr float AUTOROTATION_PID_KP = 300.0f;
static constexpr float AUTOROTATION_PID_KD = 10.0f;
static constexpr float AUTOROTATION_PID_MAX_P = 400.0f;
static constexpr float AUTOROTATION_PID_MAX_D = 300.0f;
static constexpr float AUTOROTATION_PID_MAX_OUTPUT = AUTOROTATION_PID_MAX_P;
static constexpr float AUTOROTATION_MIN_SMOOTHING_ALPHA = 0.001f;

/**
 * Speed at which the chassis switches from symmetrical driving to diagonal driving, for a holonomic
 * X-Drive (m/s)
 */
static constexpr float AUTOROTATION_DIAGONAL_SPEED = 0.0f;

#if defined(TARGET_STANDARD_NULL)
static constexpr float WHEEL_RADIUS = 0.1016;             // 6in wheel dia
static constexpr float WHEELBASE_RADIUS = 226 / 1000.0f;  // m
inline constexpr bool WHEELBASE_MOTOR_INVERTED = false;

inline constexpr tap::algorithms::SmoothPidConfig WHEEL_VELOCITY_PID_CONFIG = {
    .kp = 300.0f,
    .ki = 14.0f,
    .kd = 0.1f,
    .maxICumulative = 1000.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C620,
    .errDeadzone = 0.5f,
    .smoothDeadzone = true,
    .antiSaturation = true,
};

#elif defined(TARGET_STANDARD_PHOBOS)
static constexpr float WHEEL_RADIUS = 0.0762;             // 4in wheel dia
static constexpr float WHEELBASE_RADIUS = 185 / 1000.0f;  // m
inline constexpr bool WHEELBASE_MOTOR_INVERTED = true;

inline constexpr tap::algorithms::SmoothPidConfig WHEEL_VELOCITY_PID_CONFIG = {
    .kp = 300.0f,
    .ki = 14.0f,
    .kd = 0.1f,
    .maxICumulative = 2000.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C620,
    .errDeadzone = 0.5f,
    .smoothDeadzone = true,
    .antiSaturation = true,
};

#endif

/*
 * Gimbal offset from the center of the chassis, see note above for explanation of x and y (m).
 */
static constexpr float GIMBAL_X_OFFSET = 0.0f;
/**
 * @see `GIMBAL_X_OFFSET`.
 */
static constexpr float GIMBAL_Y_OFFSET = 0.0f;
static constexpr float MAX_3508_PRE_GEARBOX_RPM = 482.0f * (1 / (187.0f / 3591.0f));
static constexpr float MAX_CHASSIS_WHEEL_SPEED_RPM =
    MAX_3508_PRE_GEARBOX_RPM *
    CHASSIS_GEARBOX_RATIO;  // Max theoretical M3508 wheel speed post gearbox

static constexpr BeybladeConfig BEYBLADE_CONFIG{
    .beybladeRotationalSpeedFractionOfMax = 0.9f,
    .beybladeTranslationalSpeedMultiplier = 0.6f,
    .beybladeRotationalSpeedMultiplierWhenTranslating = 0.8f,
    .translationalSpeedThresholdMultiplierForRotationSpeedDecrease = 0.05f,
    .beybladeRampRate = 50,
};

static constexpr float INITIAL_CHASSIS_POSITION_X = 0.75f;
static constexpr float INITIAL_CHASSIS_POSITION_Y = 4.0f;

static constexpr tap::motor::MotorId RIGHT_FRONT_MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::motor::MotorId LEFT_FRONT_MOTOR_ID = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId LEFT_BACK_MOTOR_ID = tap::motor::MOTOR3;
static constexpr tap::motor::MotorId RIGHT_BACK_MOTOR_ID = tap::motor::MOTOR4;
static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS2;
}  // namespace aruwsrc::control::chassis

#endif  // STANDARD_CHASSIS_CONSTANTS_HPP_
