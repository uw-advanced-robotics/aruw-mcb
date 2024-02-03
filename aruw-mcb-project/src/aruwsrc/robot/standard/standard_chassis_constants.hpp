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

#include "aruwsrc/control/chassis/new-chassis/wheel.hpp"
#include "modm/math/filter/pid.hpp"
#include "modm/math/interpolation/linear.hpp"

// Do not include this file directly: use chassis_constants.hpp instead.
#ifndef CHASSIS_CONSTANTS_HPP_
#error "Do not include this file directly! Use chassis_constants.hpp instead."
#endif

namespace aruwsrc::chassis
{
/**
 * Maps max power (in Watts) to max chassis wheel speed (RPM).
 */
static constexpr modm::Pair<int, float> CHASSIS_POWER_TO_MAX_SPEED_LUT[] = {
    {50, 4'500},
    {60, 5'700},
    {70, 6'400},
    {80, 6'700},
    {100, 7'000},
    {120, 8'000},
};

static modm::interpolation::Linear<modm::Pair<int, float>> CHASSIS_POWER_TO_SPEED_INTERPOLATOR(
    CHASSIS_POWER_TO_MAX_SPEED_LUT,
    MODM_ARRAY_SIZE(CHASSIS_POWER_TO_MAX_SPEED_LUT));

/**
 * The minimum desired wheel speed for chassis rotation when translational scaling via
 * calculateRotationTranslationalGain is performed.
 */
static constexpr float MIN_ROTATION_THRESHOLD = 800.0f;

/**
 * Pin to use for current sensing
 */
static constexpr tap::gpio::Analog::Pin CURRENT_SENSOR_PIN = tap::gpio::Analog::Pin::S;

/// @see power_limiter.hpp for what these mean
static constexpr float STARTING_ENERGY_BUFFER = 60.0f;
static constexpr float ENERGY_BUFFER_LIMIT_THRESHOLD = 60.0f;
static constexpr float ENERGY_BUFFER_CRIT_THRESHOLD = 10.0f;

static constexpr float VELOCITY_PID_KP = 20.0f;
static constexpr float VELOCITY_PID_KI = 0.2f;
static constexpr float VELOCITY_PID_KD = 0.0f;
static constexpr float VELOCITY_PID_MAX_ERROR_SUM = 5'000.0f;
/**
 * This max output is measured in the c620 robomaster translated current.
 * Per the datasheet, the controllable current range is -16384 ~ 0 ~ 16384.
 * The corresponding speed controller output torque current range is
 * -20 ~ 0 ~ 20 A.
 */
static constexpr float VELOCITY_PID_MAX_OUTPUT = 16'000.0f;

static const SmoothPidConfig MOTOR_PID_CONFIG = {
    aruwsrc::chassis::VELOCITY_PID_KP,
    aruwsrc::chassis::VELOCITY_PID_KI,
    aruwsrc::chassis::VELOCITY_PID_KD,
    aruwsrc::chassis::VELOCITY_PID_MAX_ERROR_SUM,
    aruwsrc::chassis::VELOCITY_PID_MAX_OUTPUT};

/**
 * Rotation PID: A PD controller for chassis autorotation.
 */
static constexpr float AUTOROTATION_PID_KP = 5'729.6f;
static constexpr float AUTOROTATION_PID_KD = 57.3f;
static constexpr float AUTOROTATION_PID_MAX_P = 4'000.0f;
static constexpr float AUTOROTATION_PID_MAX_D = 5'000.0f;
static constexpr float AUTOROTATION_PID_MAX_OUTPUT = 5'500.0f / 10;
static constexpr float AUTOROTATION_MIN_SMOOTHING_ALPHA = 0.001f;

/**
 * Speed at which the chassis switches from symmetrical driving to diagonal driving, for a holonomic
 * X-Drive (m/s) NOT USEFUL FOR STANDARDS
 */
static constexpr float AUTOROTATION_DIAGONAL_SPEED = 0.0f;

/**
 * Radius of the wheels (m).
 */
static constexpr float WHEEL_RADIUS = 0.076;
static constexpr float CHASSIS_GEARBOX_RATIO = (187.0f / 3591.0f);
/*
 * Wheel orientation (chassis relative) (rad)
 */
static constexpr float WHEEL_ORIENTATION_CHASSIS_RELATIVE = 0.0f;

#ifdef TARGET_STANDARD_WOODY
/**
 * Distance from center of the two front wheels (m).
 */
static constexpr float WIDTH_BETWEEN_WHEELS_Y = 0.366f;
/**
 * Distance from center of the front and rear wheels (m).
 */
static constexpr float WIDTH_BETWEEN_WHEELS_X = 0.366f;

#elif defined(TARGET_STANDARD_ELSA)

static constexpr float WIDTH_BETWEEN_WHEELS_Y = 0.340f;
static constexpr float WIDTH_BETWEEN_WHEELS_X = 0.374f;

#elif defined(TARGET_STANDARD_SPIDER)

static constexpr float WIDTH_BETWEEN_WHEELS_Y = 0.385f;
static constexpr float WIDTH_BETWEEN_WHEELS_X = 0.366f;

#endif

static const aruwsrc::chassis::WheelConfig LEFT_FRONT_MECANUM_WHEEL_CONFIG = {
    WIDTH_BETWEEN_WHEELS_X / 2,
    WIDTH_BETWEEN_WHEELS_Y / 2,
    WHEEL_ORIENTATION_CHASSIS_RELATIVE,
    WHEEL_RADIUS * 2,
    1.0f,
    CHASSIS_GEARBOX_RATIO,
    MOTOR_PID_CONFIG,
    1000.0f,
    true};
static const aruwsrc::chassis::WheelConfig RIGHT_FRONT_MECANUM_WHEEL_CONFIG = {
    WIDTH_BETWEEN_WHEELS_X / 2,
    -WIDTH_BETWEEN_WHEELS_Y / 2,
    WHEEL_ORIENTATION_CHASSIS_RELATIVE,
    WHEEL_RADIUS * 2,
    1.0f,
    CHASSIS_GEARBOX_RATIO,
    MOTOR_PID_CONFIG,
    1000.0f,
    true};
static const aruwsrc::chassis::WheelConfig LEFT_BACK_MECANUM_WHEEL_CONFIG = {
    -WIDTH_BETWEEN_WHEELS_X / 2,
    WIDTH_BETWEEN_WHEELS_Y / 2,
    WHEEL_ORIENTATION_CHASSIS_RELATIVE,
    WHEEL_RADIUS * 2,
    1.0f,
    CHASSIS_GEARBOX_RATIO,
    MOTOR_PID_CONFIG,
    1000.0f,
    true};
static const aruwsrc::chassis::WheelConfig RIGHT_BACK_MECANUM_WHEEL_CONFIG = {
    -WIDTH_BETWEEN_WHEELS_X / 2,
    -WIDTH_BETWEEN_WHEELS_Y / 2,
    WHEEL_ORIENTATION_CHASSIS_RELATIVE,
    WHEEL_RADIUS * 2,
    1.0f,
    CHASSIS_GEARBOX_RATIO,
    MOTOR_PID_CONFIG,
    1000.0f,
    true};
static constexpr float WHEELBASE_HYPOTENUSE =
    (WIDTH_BETWEEN_WHEELS_X + WIDTH_BETWEEN_WHEELS_Y == 0)
        ? 1
        : 2 / (WIDTH_BETWEEN_WHEELS_X + WIDTH_BETWEEN_WHEELS_Y);

/*
 * Gimbal offset from the center of the chassis, see note above for explanation of x and y (m).
 */
static constexpr float GIMBAL_X_OFFSET = 0.0f;
/**
 * @see `GIMBAL_X_OFFSET`.
 */
static constexpr float GIMBAL_Y_OFFSET = 0.0f;

/**
 * Fraction of max chassis speed that will be applied to rotation when beyblading
 */
static constexpr float BEYBLADE_ROTATIONAL_SPEED_FRACTION_OF_MAX = 0.75f;

/**
 * Fraction between [0, 1], what we multiply user translational input by when beyblading.
 */
static constexpr float BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER = 0.6f;

/**
 * Threshold, a fraction of the maximum translational speed that is used to determine if beyblade
 * speed should be reduced (when translating at an appreciable speed beyblade speed is reduced).
 */
static constexpr float
    BEYBLADE_TRANSLATIONAL_SPEED_THRESHOLD_MULTIPLIER_FOR_ROTATION_SPEED_DECREASE = 0.5f;

/**
 * The fraction to cut rotation speed while moving and beyblading
 */
static constexpr float BEYBLADE_ROTATIONAL_SPEED_MULTIPLIER_WHEN_TRANSLATING = 0.7f;
/**
 * Rotational speed to update the beyblade ramp target by each iteration until final rotation
 * setpoint reached, in RPM.
 */
static constexpr float BEYBLADE_RAMP_UPDATE_RAMP = 50;
}  // namespace aruwsrc::chassis

#endif  // STANDARD_CHASSIS_CONSTANTS_HPP_
