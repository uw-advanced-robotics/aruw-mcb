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

#ifndef DRONE_TURRET_CONSTANTS_HPP_
#define DRONE_TURRET_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/sensors/encoder/can_encoder/can_encoder.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/turret/algorithms/turret_gravity_compensation.hpp"
#include "aruwsrc/control/turret/turret_motor_config.hpp"
#include "modm/math/geometry/angle.hpp"

// Do not include this file directly: use turret_constants.hpp instead.
#ifndef TURRET_CONSTANTS_HPP_
#error "Do not include this file directly! Use turret_controller_constants.hpp instead."
#endif

namespace aruwsrc::control::turret
{
static constexpr uint8_t NUM_TURRETS = 1;

static constexpr float USER_YAW_INPUT_SCALAR = -0.01f;
static constexpr float USER_PITCH_INPUT_SCALAR = -0.01f;
static constexpr float DAMIAO_4310_VELOCITY_KP_RAD_PER_SEC = 80.0f * 60.0f / M_TWOPI;
static constexpr float DAMIAO_4310_MAX_OUTPUT_MILLI_NM = 2000.0f;

static constexpr tap::can::CanBus CAN_BUS_YAW_MOTOR = tap::can::CanBus::CAN_BUS1;
static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR5;
static constexpr tap::can::CanBus CAN_BUS_YAW_ENCODER = tap::can::CanBus::CAN_BUS2;
static constexpr tap::encoder::CanEncoderId YAW_ENCODER_ID = tap::encoder::CanEncoderId::ID1;
static constexpr float YAW_ENCODER_TO_TURRET_RATIO = 1.0f / 2.0f;
static constexpr uint32_t YAW_ENCODER_CURRENT_MIN_POSITION = 400;
static constexpr uint32_t YAW_ENCODER_CURRENT_MAX_POSITION = 3398;
static constexpr uint32_t YAW_ENCODER_CURRENT_CENTER_POSITION =
    (YAW_ENCODER_CURRENT_MIN_POSITION +
     ((YAW_ENCODER_CURRENT_MAX_POSITION - YAW_ENCODER_CURRENT_MIN_POSITION) / 2)) %
    tap::encoder::CanEncoder::ENCODER_RESOLUTION;
static constexpr uint32_t YAW_ENCODER_OFFSET_SHIFT =
    (YAW_ENCODER_CURRENT_MAX_POSITION +
     ((tap::encoder::CanEncoder::ENCODER_RESOLUTION - YAW_ENCODER_CURRENT_MAX_POSITION +
       YAW_ENCODER_CURRENT_MIN_POSITION) /
      2)) %
    tap::encoder::CanEncoder::ENCODER_RESOLUTION;
static constexpr uint32_t YAW_ENCODER_HOME_POSITION = YAW_ENCODER_OFFSET_SHIFT;
static constexpr uint32_t YAW_ENCODER_CENTER_POSITION =
    (YAW_ENCODER_CURRENT_CENTER_POSITION + tap::encoder::CanEncoder::ENCODER_RESOLUTION -
     YAW_ENCODER_OFFSET_SHIFT) %
    tap::encoder::CanEncoder::ENCODER_RESOLUTION;
static constexpr uint32_t YAW_ENCODER_MIN_POSITION =
    (YAW_ENCODER_CURRENT_MIN_POSITION + tap::encoder::CanEncoder::ENCODER_RESOLUTION -
     YAW_ENCODER_OFFSET_SHIFT) %
    tap::encoder::CanEncoder::ENCODER_RESOLUTION;
static constexpr uint32_t YAW_ENCODER_MAX_POSITION =
    (YAW_ENCODER_CURRENT_MAX_POSITION + tap::encoder::CanEncoder::ENCODER_RESOLUTION -
     YAW_ENCODER_OFFSET_SHIFT) %
    tap::encoder::CanEncoder::ENCODER_RESOLUTION;
static constexpr float YAW_START_ANGLE = -static_cast<float>(YAW_ENCODER_CENTER_POSITION) /
                                         tap::encoder::CanEncoder::ENCODER_RESOLUTION * M_TWOPI *
                                         YAW_ENCODER_TO_TURRET_RATIO;
static constexpr float YAW_MIN_ANGLE = static_cast<float>(YAW_ENCODER_MIN_POSITION) /
                                           tap::encoder::CanEncoder::ENCODER_RESOLUTION * M_TWOPI *
                                           YAW_ENCODER_TO_TURRET_RATIO +
                                       YAW_START_ANGLE + modm::toRadian(15);
static constexpr float YAW_MAX_ANGLE = static_cast<float>(YAW_ENCODER_MAX_POSITION) /
                                           tap::encoder::CanEncoder::ENCODER_RESOLUTION * M_TWOPI *
                                           YAW_ENCODER_TO_TURRET_RATIO +
                                       YAW_START_ANGLE - modm::toRadian(15);

static constexpr tap::can::CanBus CAN_BUS_PITCH_MOTOR = tap::can::CanBus::CAN_BUS1;
static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR7;
static constexpr float PITCH_DAMIAO_STRAIGHT_DOWN_POSITION = -0.975624084f;
static constexpr float PITCH_DAMIAO_FIRST_LIMIT_POSITION = -2.63199043f;
static constexpr float PITCH_DAMIAO_OPPOSITE_LIMIT_POSITION = 0.349622726f;
static constexpr float PITCH_START_ANGLE = PITCH_DAMIAO_STRAIGHT_DOWN_POSITION;
static constexpr float PITCH_MIN_ANGLE =
    PITCH_DAMIAO_FIRST_LIMIT_POSITION + PITCH_START_ANGLE + modm::toRadian(2);
static constexpr float PITCH_MAX_ANGLE =
    PITCH_DAMIAO_OPPOSITE_LIMIT_POSITION + PITCH_START_ANGLE - modm::toRadian(2);
static constexpr float PITCH_IMU_CALIBRATION_ANGLE = -2.45574856;

static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = YAW_START_ANGLE,
    .startEncoderValue = YAW_ENCODER_HOME_POSITION,
    .minAngle = YAW_MIN_ANGLE,
    .maxAngle = YAW_MAX_ANGLE,
    .limitMotorAngles = true,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 0,
    .minAngle = PITCH_DAMIAO_FIRST_LIMIT_POSITION,
    .maxAngle = PITCH_DAMIAO_STRAIGHT_DOWN_POSITION,
    .limitMotorAngles = true,
};

static constexpr algorithms::TurretGravitationalForceOffset::TurretGravityParams  // TODO tune
    TURRET_GRAVITY_CONFIG{.cgX = 20.0f, .cgZ = 16.5f, .gravityCompensatorMax = -5200.0f};

// static const tap::algorithms::transforms::Transform TURRET_IMU_MOUNTING_TRANSFORM(
//     0.0f,
//     0.0f,
//     0.0f,
//     0.0f,
//     M_PI,
//     -M_PI_2);

static const tap::algorithms::transforms::Transform TURRET_IMU_CALIBRATION_MOUNTING_TRANSFORM(
    0.0f,
    0.0f,
    0.0f,
    M_PI,
    0.0f,
    -M_PI_2);

namespace world_rel_turret_imu
{
static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_CONFIG = {
    .kp = 15.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 10.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_VEL_PID_CONFIG = {
    .kp = 3000.0f,
    .ki = 0.0f,
    .kd = 0.2f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C610,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_CONFIG = {
    .kp = 24.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 40.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_VEL_PID_CONFIG = {
    .kp = DAMIAO_4310_VELOCITY_KP_RAD_PER_SEC,
    .ki = 2500.0f,
    .kd = 0.0f,
    .maxICumulative = 7000.0f,
    .maxOutput = DAMIAO_4310_MAX_OUTPUT_MILLI_NM,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
}  // namespace world_rel_turret_imu

namespace chassis_rel
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 3000.0f,
    .ki = 0.0f,
    .kd = 200.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C610,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 60.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    .kp = 7000.0f,
    .ki = 2500.0f,
    .kd = DAMIAO_4310_VELOCITY_KP_RAD_PER_SEC,
    .maxICumulative = 7000.0f,
    .maxOutput = DAMIAO_4310_MAX_OUTPUT_MILLI_NM,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 20.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 2.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
}  // namespace chassis_rel

}  // namespace aruwsrc::control::turret

#endif  // DRONE_TURRET_CONSTANTS_HPP_
