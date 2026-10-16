/*
 * Copyright (c) 2021-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SENTRY_CHASSIS_CONSTANTS_HPP_
#define SENTRY_CHASSIS_CONSTANTS_HPP_

#include <cmath>

#include "tap/algorithms/transforms/transform.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/chassis/beyblade_config.hpp"
#include "aruwsrc/control/chassis/swerve_module_config.hpp"

namespace aruwsrc::control::chassis
{
static constexpr float CAP_BANK_CAPACITANCE = 6.66f;
static constexpr int CAP_BANK_MAX_AVAILABLE_POWER = 100;  // watts
// Initial position of the chassis in the field (meters)
static constexpr float INITIAL_CHASSIS_POSITION_X = 0.75f;
static constexpr float INITIAL_CHASSIS_POSITION_Y = 4.0f;

/**
 * Maps max power (in Watts) to max chassis wheel speed (RPM).
 */
static constexpr modm::Pair<int, float> CHASSIS_POWER_TO_MAX_SPEED_LUT[] = {
    {1, 370},
    {2, 375}};  // TODO: TUNE!

static modm::interpolation::Linear<modm::Pair<int, float>> CHASSIS_POWER_TO_SPEED_INTERPOLATOR(
    CHASSIS_POWER_TO_MAX_SPEED_LUT,
    MODM_ARRAY_SIZE(CHASSIS_POWER_TO_MAX_SPEED_LUT));

/**
 * The minimum desired wheel speed for chassis rotation when translational scaling via
 * calculateRotationTranslationalGain is performed.
 */
static constexpr float MIN_ROTATION_THRESHOLD = 40.0f;

/// @see power_limiter.hpp for what these mean
static constexpr float STARTING_ENERGY_BUFFER = 60.0f;
static constexpr float ENERGY_BUFFER_LIMIT_THRESHOLD = 60.0f;
static constexpr float ENERGY_BUFFER_CRIT_THRESHOLD = 10.0f;

static constexpr float VELOCITY_PID_KV = 0.0f;
static constexpr float VELOCITY_PID_KS = 0.0f;
static constexpr tap::algorithms::SmoothPidConfig WHEEL_VELOCITY_PID_CONFIG = {
    .kp = 300.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 2000.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C620,
    .errDeadzone = 1.0f,
};

/**
 * Rotation PID: A PD controller for chassis autorotation. The PID parameters for the
 * controller are listed below.
 */
static constexpr float AUTOROTATION_PID_KP = 5'729.6f;
static constexpr float AUTOROTATION_PID_KD = 57.3f;
static constexpr float AUTOROTATION_PID_MAX_P = 5000.0f;
static constexpr float AUTOROTATION_PID_MAX_D = 5000.0f;
static constexpr float AUTOROTATION_PID_MAX_OUTPUT = 5500.0f;
static constexpr float AUTOROTATION_MIN_SMOOTHING_ALPHA = 0.001f;

/**
 * Speed at which the chassis switches from symmetrical driving to diagonal driving, for a holonomic
 * X-Drive (m/s)
 */
static constexpr float AUTOROTATION_DIAGONAL_SPEED = 0.0f;

// mechanical chassis constants
/**
 * Radius of the wheels (m)
 */
static constexpr float FUDGE_FACTOR = 0.999141881817f;
static constexpr float WHEEL_RADIUS = 0.0762f * FUDGE_FACTOR;

static constexpr float WHEELBASE_RADIUS = 0.23864f;

/**
 * Gimbal offset from the center of the chassis, see note above for explanation of x and y.
 */
static constexpr float GIMBAL_X_OFFSET = 0.0f;
/**
 * @see `GIMBAL_X_OFFSET`.
 */
static constexpr float GIMBAL_Y_OFFSET = 0.0f;
static constexpr float CHASSIS_GEARBOX_RATIO = (17.0f / 268.0f);

static constexpr BeybladeConfig BEYBLADE_CONFIG{
    .beybladeRotationalSpeedFractionOfMax = 0.9f,
    .beybladeTranslationalSpeedMultiplier = 0.5f,
    .beybladeRotationalSpeedMultiplierWhenTranslating = 0.8f,
    .translationalSpeedThresholdMultiplierForRotationSpeedDecrease = 0.25f,
    .beybladeRampRate = 100,
};

// The cap bank only sprints above both of these thresholds; otherwise it recharges.
static constexpr float CAP_BANK_SPRINT_TRANSLATIONAL_VELOCITY_THRESHOLD = 0.25f;  // m/s
static constexpr float CAP_BANK_SPRINT_ENERGY_THRESHOLD = 50.0f;                  // J

static constexpr tap::motor::MotorId LEFT_FRONT_MOTOR_ID = tap::motor::MOTOR4;
static constexpr tap::motor::MotorId LEFT_BACK_MOTOR_ID = tap::motor::MOTOR3;
static constexpr tap::motor::MotorId RIGHT_BACK_MOTOR_ID = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId RIGHT_FRONT_MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS2;

namespace chassisImu
{
inline const tap::algorithms::transforms::Transform CHASSIS_MCB_BMI088_MOUNTING_TRANSFORM(
    0.04117f,
    0.0f,
    0.04945f,
    0.0f,
    0.0f,
    0.0f);

inline const tap::algorithms::transforms::Transform CHASSIS_MCB_ISM330_MOUNTING_TRANSFORM(
    0.09811f,
    0.0f,
    0.003253f,
    0.0f,
    0.0f,
    0.0f);
}  // namespace chassisImu
}  // namespace aruwsrc::control::chassis
#endif  // SENTRY_CHASSIS_CONSTANTS_HPP_
