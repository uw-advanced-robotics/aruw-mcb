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

#ifndef ENGINEER_CHASSIS_CONSTANTS_HPP_
#define ENGINEER_CHASSIS_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/gpio/analog.hpp"

#include "aruwsrc/control/chassis/beyblade_config.hpp"
#include "modm/math/interpolation/linear.hpp"
#include "tap/algorithms/transforms/position.hpp"

// Do not include this file directly: use chassis_constants.hpp instead.
#ifndef CHASSIS_CONSTANTS_HPP_
#error "Do not include this file directly! Use chassis_constants.hpp instead."
#endif

namespace aruwsrc::control::chassis
{
// Initial position of the chassis in the field (meters)
static constexpr float INITIAL_CHASSIS_POSITION_X = 0.0f;  // TODO: find initial position of chassis
static constexpr float INITIAL_CHASSIS_POSITION_Y = 0.0f;  // TODO: find initial position of chassis

// Initial orientation of the chassis in the field (radians)
static constexpr float INITIAL_CHASSIS_ORIENTATION =
    0.0f;  // TODO: find initial orientation of chassis

/**
 * Maps max power (in Watts) to max chassis wheel speed (RPM).
 *
 * Since the engineer has no power limiting, this lookup table doesn't matter much, just set some
 * high values.
 */
static constexpr modm::Pair<int, float> CHASSIS_POWER_TO_MAX_SPEED_LUT[] = {{1, 8'000}, {1, 8'000}};

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

static constexpr float VELOCITY_PID_KP = 10.0f;
static constexpr float VELOCITY_PID_KI = 0.0f;
static constexpr float VELOCITY_PID_KD = 0.0f;
static constexpr float VELOCITY_PID_MAX_ERROR_SUM = 0.0f;
static constexpr float VELOCITY_PID_KV = 0.0f;
static constexpr float VELOCITY_PID_KS = 0.0;

/**
 * This max output is measured in the c620 robomaster translated current.
 * Per the datasheet, the controllable current range is -16384 ~ 0 ~ 16384.
 * The corresponding speed controller output torque current range is
 * -20 ~ 0 ~ 20 A.
 */
static constexpr float VELOCITY_PID_MAX_OUTPUT = 16'000.0f;

static constexpr tap::algorithms::SmoothPidConfig WHEEL_VELOCITY_PID_CONFIG = {
    .kp = VELOCITY_PID_KP,
    .ki = VELOCITY_PID_KI,
    .kd = VELOCITY_PID_KD,
    .maxICumulative = VELOCITY_PID_MAX_ERROR_SUM,
    .maxOutput = VELOCITY_PID_MAX_OUTPUT,
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
 * X-Drive (m/s) NOT USEFUL FOR ENGINEER
 */
static constexpr float AUTOROTATION_DIAGONAL_SPEED = 0.0f;

// mechanical chassis constants
/**
 * Radius of the wheels (m)
 */
static constexpr float WHEEL_RADIUS = 0.076f;
/**
 * Radius of the deadwheels (m)
 */
static constexpr float DEADWHEEL_RADIUS = 0.0f;  // TODO: measue radius of deadwheels.
/**
 * Distance from the center axis of the robot to each deadwheel (m)
 */
static constexpr float parallelOneCenterToWheelDistance =
    0.0f;  // TODO: measure distance from center to parallel deadwheel one.
static constexpr float parallelTwoCenterToWheelDistance =
    0.0f;  // TODO: measure distance from center to odomFrameToRobotFrame deadwheel two.
static constexpr float perpendicularCenterToWheelDistance =
    0.0f;  // TODO: measure distance from center to the perpendiculatr deadwheel.
/**
 * Relative orientation of dead wheels (rad)
 */
static constexpr float odomFrameToRobotFrame =
    0.0f;  // TODO: measure distance from center to deadwheel one.
/**
 * Distance from center of the two front wheels (m)
 */
static constexpr float WIDTH_BETWEEN_WHEELS_Y = 0.46f;
/**
 * Distance from center of the front and rear wheels (m).
 */
static constexpr float WIDTH_BETWEEN_WHEELS_X = 0.46f;

static constexpr float WHEELBASE_RADIUS = 0.46f;

/**
 * Gimbal offset from the center of the chassis, see note above for explanation of x and y.
 */
static constexpr float GIMBAL_X_OFFSET = 0.0f;
/**
 * @see `GIMBAL_X_OFFSET`.
 */
static constexpr float GIMBAL_Y_OFFSET = 0.0f;
static constexpr float CHASSIS_GEARBOX_RATIO = (1.0f / 19.0f);

static constexpr BeybladeConfig BEYBLADE_CONFIG{
    .beybladeRotationalSpeedFractionOfMax = 0.75f,
    .beybladeTranslationalSpeedMultiplier = 0.5f,
    .beybladeRotationalSpeedMultiplierWhenTranslating = 0.5f,
    .translationalSpeedThresholdMultiplierForRotationSpeedDecrease = 0.5f,
    .beybladeRampRate = 100,
};

/**
 * Engineer auto nav path
 */
static const tap::algorithms::transforms::Position ENGINEER_AUTO_NAV_START_POSITION = Position(0.5f, 0.5f);  // TODO: find actual start position
static const float ENGINEER_AUTO_NAV_MARIGIN = 0.05f; // Addition marigin between target above robot width
static const float ENGINEER_AUTO_NAV_POINT_OFFSET_X = WIDTH_BETWEEN_WHEELS_X / 2 + ENGINEER_AUTO_NAV_MARIGIN;
static const tap::algorithms::transforms::Position ENGINEER_AUTO_NAV_PATH_POINTS[] = {
    ENGINEER_AUTO_NAV_START_POSITION,
    Position(0.600f + ENGINEER_AUTO_NAV_POINT_OFFSET_X, 3.555f), // First cube pickup
    Position(3.870f - ENGINEER_AUTO_NAV_POINT_OFFSET_X, 0.540f), // First cube dropoff
    Position(0.600f + ENGINEER_AUTO_NAV_POINT_OFFSET_X, 3.825f), // Second cube pickup
    Position(3.870f - ENGINEER_AUTO_NAV_POINT_OFFSET_X, 1.620f), // Second cube dropoff
    Position(0.600f + ENGINEER_AUTO_NAV_POINT_OFFSET_X, 4.095f), // Third cube pickup
    Position(3.870f - ENGINEER_AUTO_NAV_POINT_OFFSET_X, 2.700f), // Third cube dropoff
};

static constexpr float CHASSIS_SPEED_DIVSOR_NORMAL = 3.5;
static constexpr float CHASSIS_SPEED_DIVSOR_SPRINT = 8;
}  // namespace aruwsrc::control::chassis

#endif  // ENGINEER_CHASSIS_CONSTANTS_HPP_