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

#ifndef HERO_TURRET_CONSTANTS_HPP_
#define HERO_TURRET_CONSTANTS_HPP_

#include "tap/algorithms/fuzzy_pd.hpp"
#include "tap/communication/sensors/encoder/can_encoder/can_encoder.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/turret/algorithms/turret_gravity_compensation.hpp"
#include "aruwsrc/control/turret/algorithms/turret_spring_compensation.hpp"
#include "aruwsrc/control/turret/turret_motor_config.hpp"
#include "aruwsrc/robot/hero/hero_pitch_turret_motor.hpp"
#include "modm/container/pair.hpp"
#include "modm/math/geometry/angle.hpp"

// Do not include this file directly: use turret_constants.hpp instead.
#ifndef TURRET_CONSTANTS_HPP_
#error "Do not include this file directly! Use turret_controller_constants.hpp instead."
#endif

namespace aruwsrc::control::turret
{
inline const tap::algorithms::transforms::Transform TURRET_MCB_BMI088_MOUNTING_TRANSFORM(
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    PI);
inline const tap::algorithms::transforms::Transform TURRET_MCB_ISM330_MOUNTING_TRANSFORM(
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    -M_2_PI);

static constexpr uint8_t NUM_TURRETS = 1;

static constexpr float USER_YAW_INPUT_SCALAR = 0.02f;
static constexpr float USER_PITCH_INPUT_SCALAR = 0.02f;

static constexpr tap::can::CanBus CAN_BUS_YAW_MOTOR = tap::can::CanBus::CAN_BUS2;
static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR5;

static constexpr tap::encoder::CanEncoderId YAW_LAMPREY_ENCODER_ID =
    tap::encoder::CanEncoderId::ID7;
static constexpr tap::can::CanBus YAW_LAMPREY_ENCODER_CAN_BUS = tap::can::CanBus::CAN_BUS2;

static constexpr float YAW_LAMPREY_RATIO = 1.0f;
static constexpr float YAW_LAMPREY_ENCODER_HOME_POSITION = -modm::toRadian(110.0f);

static constexpr tap::encoder::CanEncoderId YAW_ENCODER_ID = tap::encoder::CanEncoderId::ID0;

static constexpr tap::can::CanBus YAW_ENCODER_CAN_BUS = tap::can::CanBus::CAN_BUS2;

static constexpr float YAW_ENCODER_RATIO = 1.0f;  // One for use in binned alignment

static constexpr float BINNED_ALIGNMENT_OFFSET = 2.44f;

static const modm::Pair<float, float> LAMPREY_LUT[0] = {};

static constexpr tap::can::CanBus CAN_BUS_PITCH_MOTOR = tap::can::CanBus::CAN_BUS1;
static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR5;

static constexpr uint32_t ENCODER_RATIO_NUM = 1;
static constexpr uint32_t ENCODER_RATIO_DEN = 2;

static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 414,
    .minAngle = 0,         ///< Doesn't matter since yaw not limited
    .maxAngle = 2 * M_PI,  ///< Doesn't matter since yaw not limited
    .limitMotorAngles = false,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 1975,
    .minAngle = -.32f,
    .maxAngle = 0.56f,
    .limitMotorAngles = true,
};

inline constexpr aruwsrc::hero::HeroPitchLinkage::FourBarLinkageConfig PITCH_LINKAGE_CONFIG = {
    .l1 = 0.080f,  // 80mm fixed link
    .l2 = 0.080f,  // turret head
    .l3 = 0.120f,  // longer linkage
    .l4 = 0.040f,  // shorter linkage
};

// Turret is perfectly balanced
static constexpr algorithms::TurretGravitationalForceOffset::TurretGravityParams
    TURRET_GRAVITY_CONFIG{.cgX = 12.1f, .cgZ = -15.67f, .gravityCompensatorMax = -8000.0f};

static constexpr algorithms::TurretSpringForceOffset::TurretSpringParams TURRET_SPRING_CONFIG{
    .turretPitchMountX = 0.0f,
    .turretPitchMountZ = 0.0f,
    .turretYawMountX = 0.0f,
    .turretYawMountZ = 0.0f,
    .springConstant = 0.0f,
    .springFreeLength = 0.0f,
};

namespace world_rel_turret_imu
{
static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_CONFIG = {
    .kp = 50.0f,
    .ki = 0.0f,
    .kd = 1.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 15.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_AUTO_AIM_CONFIG = {
    .kp = 50.0f,
    .ki = 0.0f,
    .kd = 1.0f,
    .maxICumulative = 10.0f,
    .maxOutput = 15.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 0.1f,
    .tRProportionalKalman = 0.4f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_VEL_PID_CONFIG = {
    .kp = 1500.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C620,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_CONFIG = {
    .kp = 20.0f,
    .ki = 0.0f,
    .kd = 0.5f,
    .maxICumulative = 0.0f,
    .maxOutput = 20.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 30.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_AUTO_AIM_CONFIG = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 5.0f,
    .maxOutput = 2000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 30.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_VEL_PID_CONFIG = {
    .kp = 8000.0f,
    .ki = 0.0f,
    .kd = 1000.0f,
    .maxICumulative = 5'000.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
}  // namespace world_rel_turret_imu

namespace world_rel_chassis_imu
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA,
    .tQDerivativeKalman = 0.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 0.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

}

namespace chassis_rel
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 100'000.0f,
    .ki = 50'000.0f,
    .kd = 10'000.0f,
    .maxICumulative = 10'000.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.015f,
    .antiSaturation = true,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    .kp = 20'000.0f,
    .ki = 40'000.0f,
    .kd = 800.0f,
    .maxICumulative = 10000.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 400.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
    .antiSaturation = true,
};
}  // namespace chassis_rel

}  // namespace aruwsrc::control::turret

#endif  // HERO_TURRET_CONSTANTS_HPP_
