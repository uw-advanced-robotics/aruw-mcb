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
#include "tap/algorithms/transforms/position.hpp"
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
// Initial position of the chassis in the field (meters)
static constexpr float INITIAL_CHASSIS_POSITION_X =
    0.0f;  /// TODO: find initial position of chassis
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
static constexpr modm::Pair<int, float> CHASSIS_POWER_TO_MAX_SPEED_LUT[] = {{1, 200}, {2, 250}};

static modm::interpolation::Linear<modm::Pair<int, float>> CHASSIS_POWER_TO_SPEED_INTERPOLATOR(
    CHASSIS_POWER_TO_MAX_SPEED_LUT,
    MODM_ARRAY_SIZE(CHASSIS_POWER_TO_MAX_SPEED_LUT));

/**
 * The minimum desired wheel speed for chassis rotation when translational scaling via
 * calculateRotationTranslationalGain is performed.
 */
static constexpr float MIN_ROTATION_THRESHOLD = 40.0f;

/**
 * Pin to use for current sensing
 */
static constexpr tap::gpio::Analog::Pin CURRENT_SENSOR_PIN = tap::gpio::Analog::Pin::S;

/// @see power_limiter.hpp for what these mean
static constexpr float STARTING_ENERGY_BUFFER = 60.0f;
static constexpr float ENERGY_BUFFER_LIMIT_THRESHOLD = 60.0f;
static constexpr float ENERGY_BUFFER_CRIT_THRESHOLD = 10.0f;

static constexpr float VELOCITY_PID_KV = 0.0f;
static constexpr float VELOCITY_PID_KS = 0.0;

/**
 * This max output is measured in the c620 robomaster translated current.
 * Per the datasheet, the controllable current range is -16384 ~ 0 ~ 16384.
 * The corresponding speed controller output torque current range is
 * -20 ~ 0 ~ 20 A.
 */

static constexpr tap::algorithms::SmoothPidConfig WHEEL_VELOCITY_PID_CONFIG = {
    .kp = 300.0f,
    .ki = 14.0f,
    .kd = 0.0f,
    .maxICumulative = 2000.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C620,
    .errDeadzone = 1.0f,
    .smoothDeadzone = true,
};

/**
 * Rotation PID: A PD controller for chassis autorotation. The PID parameters for the
 * controller are listed below.
 */
static constexpr float AUTOROTATION_PID_KP = 200.6f;
static constexpr float AUTOROTATION_PID_KD = 10.0f;
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
static constexpr float DEADWHEEL_RADIUS = 0.016f;
/**
 * Distance from the center axis of the robot to each deadwheel (m)
 */
static constexpr float parallelOneCenterToWheelDistance = 140.975f;
static constexpr float parallelTwoCenterToWheelDistance = 140.975f;
static constexpr float perpendicularCenterToWheelDistance = 77.975f;
/**
 * Relative orientation of dead wheels (rad)
 */
static constexpr float odomFrameToRobotFrame =
    0.0f;  // TODO: measure distance from center to deadwheel one.

static constexpr float WHEELBASE_RADIUS = 0.2443f;

static constexpr float ROBOT_RADIUS = 1.0f;  // TODO: measure actual value, this one is arbitrary

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
static const tap::algorithms::transforms::Position ENGINEER_AUTO_NAV_START_POSITION =
    tap::algorithms::transforms::Position(0.5f, 0.5f, 0);  // TODO: find actual start position
static const float ENGINEER_AUTO_NAV_MARGIN =
    0.05f;  // Addition margin between target above robot width
static const float ENGINEER_AUTO_NAV_POINT_OFFSET_X = ROBOT_RADIUS + ENGINEER_AUTO_NAV_MARGIN;

static const tap::algorithms::transforms::Position FIRST_CUBE_PICKUP =
    tap::algorithms::transforms::Position(
        0.600f + ENGINEER_AUTO_NAV_POINT_OFFSET_X,
        3.555f,
        0);  // First cube pickup

static const tap::algorithms::transforms::Position FIRST_CUBE_DROPOFF =
    tap::algorithms::transforms::Position(
        3.870f - ENGINEER_AUTO_NAV_POINT_OFFSET_X,
        0.540f,
        0);  // First cube dropoff

static const tap::algorithms::transforms::Position SECOND_CUBE_PICKUP =
    tap::algorithms::transforms::Position(
        0.600f + ENGINEER_AUTO_NAV_POINT_OFFSET_X,
        3.825f,
        0);  // Second cube pickup

static const tap::algorithms::transforms::Position SECOND_CUBE_DROPOFF =
    tap::algorithms::transforms::Position(
        3.870f - ENGINEER_AUTO_NAV_POINT_OFFSET_X,
        1.620f,
        0);  // Second cube dropoff

static const tap::algorithms::transforms::Position THIRD_CUBE_PICKUP =
    tap::algorithms::transforms::Position(
        0.600f + ENGINEER_AUTO_NAV_POINT_OFFSET_X,
        4.095f,
        0);  // Third cube pickup

static const tap::algorithms::transforms::Position THIRD_CUBE_DROPOFF =
    tap::algorithms::transforms::Position(
        3.870f - ENGINEER_AUTO_NAV_POINT_OFFSET_X,
        2.700f,
        0);  // Third cube dropoff

static constexpr float CHASSIS_SPEED_DIVSOR_NORMAL = 3.5;
static constexpr float CHASSIS_SPEED_DIVSOR_SPRINT = 8;

// hardware constants, not specific to any particular chassis
static constexpr tap::motor::MotorId LEFT_FRONT_MOTOR_ID = tap::motor::MOTOR4;
static constexpr tap::motor::MotorId LEFT_BACK_MOTOR_ID = tap::motor::MOTOR3;
static constexpr tap::motor::MotorId RIGHT_BACK_MOTOR_ID = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId RIGHT_FRONT_MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS2;

}  // namespace aruwsrc::control::chassis

#endif  // ENGINEER_CHASSIS_CONSTANTS_HPP_