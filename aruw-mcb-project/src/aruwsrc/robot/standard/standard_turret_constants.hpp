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

#ifndef STANDARD_TURRET_CONSTANTS_HPP_
#define STANDARD_TURRET_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/turret/algorithms/turret_gravity_compensation.hpp"
#include "aruwsrc/control/turret/algorithms/turret_spring_compensation.hpp"
#include "aruwsrc/control/turret/algorithms/turret_stos_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_stos_turret_controller.hpp"
#include "aruwsrc/control/turret/turret_motor_config.hpp"
#include "modm/math/geometry/angle.hpp"

// Do not include this file directly: use turret_constants.hpp instead.
#ifndef TURRET_CONSTANTS_HPP_
#error "Do not include this file directly! Use turret_controller_constants.hpp instead."
#endif

namespace aruwsrc::control::turret
{
static constexpr uint8_t NUM_TURRETS = 1;

static constexpr float USER_YAW_INPUT_SCALAR = 0.02f;
static constexpr float USER_PITCH_INPUT_SCALAR = 0.02f;

#if defined(TARGET_STANDARD_NULL)
inline const tap::algorithms::transforms::Transform TURRET_MCB_BMI088_MOUNTING_TRANSFORM(
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    PI);
#else
inline const tap::algorithms::transforms::Transform TURRET_MCB_BMI088_MOUNTING_TRANSFORM(
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f);
#endif

inline const tap::algorithms::transforms::Transform TURRET_MCB_ISM330_MOUNTING_TRANSFORM(
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    PI);

static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;
#if defined(TARGET_STANDARD_NULL)
static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR6;
static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR5;
#else
static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR6;
static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR5;
#endif

#if defined(TARGET_STANDARD_NULL)
static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 8146,
    .minAngle = 0,
    .maxAngle = M_PI,
    .limitMotorAngles = false,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 8956,
    .minAngle = modm::toRadian(-18),
    .maxAngle = modm::toRadian(28),
    .limitMotorAngles = true,
};

#elif defined(TARGET_STANDARD_PHOBOS)
static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 5767,
    .minAngle = 0,
    .maxAngle = M_PI,
    .limitMotorAngles = false,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 8171,
    .minAngle = modm::toRadian(-7),
    .maxAngle = modm::toRadian(40),
    .limitMotorAngles = true,
};
#else
#error "Attempted to include standard_turret_constants.hpp for nonstandard target."
#endif

#if defined(TARGET_STANDARD_NULL)
static constexpr float TORQUE_TO_DESIRED_OUT =
    1.3f / tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA;  // 1.3Nm max torque
static constexpr float TURRET_WEIGHT_KG = 1.646f;       // 1.646kg from CAD

// Actual CAD value is 55.76, decreased for balls in hopper
static constexpr algorithms::TurretGravitationalForceOffset::TurretGravityParams
    TURRET_GRAVITY_CONFIG{
        .cgX = 33.83f,
        .cgZ = 26.68f,
        .gravityCompensatorMax = -5000.0f,
    };

static constexpr algorithms::TurretSpringForceOffset::TurretSpringParams TURRET_SPRING_CONFIG{
    .turretPitchMountX = 0.0f,
    .turretPitchMountZ = 0.0f,
    .turretYawMountX = 0.0f,
    .turretYawMountZ = 0.0f,
    .springConstant = 0.0f,
    .springFreeLength = 0.0f,
};

#elif defined(TARGET_STANDARD_PHOBOS)
static constexpr float TORQUE_TO_DESIRED_OUT =
    1.3f / tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA;  // 1.3Nm max torque
static constexpr float TURRET_WEIGHT_KG = 1.646f;       // 1.646kg from CAD

static constexpr algorithms::TurretGravitationalForceOffset::TurretGravityParams
    TURRET_GRAVITY_CONFIG{
        .cgX = 39.25f,
        .cgZ = -26.63f,
        .gravityCompensatorMax = -12'500.0f,
    };

static constexpr algorithms::TurretSpringForceOffset::TurretSpringParams TURRET_SPRING_CONFIG{
    .turretPitchMountX = -3.4f,
    .turretPitchMountZ = -37.7f,
    .turretYawMountX = 76.29f,
    .turretYawMountZ = -71.91f,
    .springConstant = -3.69f,
    .springFreeLength = 52.9f,
};

#else
#error "Attempted to include standard_turret_constants.hpp for nonstandard target."
#endif

namespace world_rel_turret_imu
{
#if defined(TARGET_STANDARD_NULL)
static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_CONFIG = {
    .kp = 50.0f,
    .ki = 0.0f,
    .kd = 5.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 100.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_AUTO_AIM_CONFIG = YAW_POS_PID_CONFIG;
// {
//     .kp = 1500.0f,
//     .ki = 0.0f,
//     .kd = 0.1875f,
//     .maxICumulative = 0.0f,
//     .maxOutput = 5000.0f,
//     .tQDerivativeKalman = 1.0f,
//     .tRDerivativeKalman = 0.0f,
//     .tQProportionalKalman = 1.0f,
//     .tRProportionalKalman = 0.0f,
//     .errDeadzone = 0.0f,
//     .errorDerivativeFloor = 0.0f,
// };

static constexpr tap::algorithms::SmoothPidConfig YAW_VEL_PID_CONFIG = {
    .kp = 4000.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_CONFIG = {
    .kp = 75.0f,
    .ki = 1.0f,
    .kd = 2.5f,
    .maxICumulative = 1.0f,
    .maxOutput = 10.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_AUTO_AIM_CONFIG =
    PITCH_POS_PID_CONFIG;
// {
//     .kp = 1500.0f,
//     .ki = 0.1f,
//     .kd = 0.3f,
//     .maxICumulative = 200.0f,
//     .maxOutput = 2000.0f,
//     .tQDerivativeKalman = 1.0f,
//     .tRDerivativeKalman = 0.0f,
//     .tQProportionalKalman = 1.0f,
//     .tRProportionalKalman = 0.0f,
//     .errDeadzone = 0.0f,
//     .errorDerivativeFloor = 0.0f,
// };

static constexpr tap::algorithms::SmoothPidConfig PITCH_VEL_PID_CONFIG = {
    .kp = 3000.0f,
    .ki = 0.0f,
    .kd = 1.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

#elif defined(TARGET_STANDARD_PHOBOS)
inline constexpr algorithms::OptimalSTOSController::STOSConstants STOS_CONSTANTS = {
    .J_TOTAL = 0.0133f,
    .TAU_MAX = 1.3f,
    .B_DAMP = 0.001f,
    .W_D = 74.6f,
    .ZETA = 0.542f,
    .SYSTEM_DELAY_SEC = 0.008f,
    .TorqueToMotorOutput = 1.0f / TORQUE_TO_DESIRED_OUT,
};

inline constexpr algorithms::TurretFeedforwardConstants FEEDFORWARD_CONSTANTS = {
    .Ka = 0.007f / TORQUE_TO_DESIRED_OUT,
    .Kv = 0.017f / TORQUE_TO_DESIRED_OUT,
    .Ks = 0.10f / TORQUE_TO_DESIRED_OUT};

static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_CONFIG = {
    .kp = 16.0f,
    .ki = 0.0f,
    .kd = 0.1f,
    .maxICumulative = 0.0f,
    .maxOutput = 40.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_AUTO_AIM_CONFIG = {
    .kp = 12'000.0f,
    .ki = 0.0f,
    .kd = 5'000.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

// tuned
static constexpr tap::algorithms::SmoothPidConfig YAW_VEL_PID_CONFIG = {
    .kp = 9000.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 16384.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_CONFIG = {
    .kp = 15.0f,
    .ki = 0.1f,
    .kd = 0.2f,
    .maxICumulative = 0.5f,
    .maxOutput = 10.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
    .smoothDeadzone = false,
    .antiSaturation = true,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_AUTO_AIM_CONFIG = {
    .kp = 30.0f,
    .ki = 100.0f,
    .kd = 0.6f,
    .maxICumulative = 1.0f,
    .maxOutput = 10.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
    .antiSaturation = true,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_VEL_PID_CONFIG = {
    .kp = 4000.0f,
    .ki = 0.0f,
    .kd = 1.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
#else
#error "Attempted to include standard_turret_constants.hpp for nonstandard target."
#endif
}  // namespace world_rel_turret_imu

namespace world_rel_chassis_imu
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 45'000.0f,
    .ki = 0.0f,
    .kd = 2'600.0f,
    .maxICumulative = 3'000.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA,
    .tQDerivativeKalman = 10.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
}  // namespace world_rel_chassis_imu

namespace chassis_rel
{
#if defined(TARGET_STANDARD_NULL)
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 25'000.0f,
    .ki = 200.0f,
    .kd = 5'000.2f,
    .maxICumulative = 5000.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 70.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    .kp = 80'000.0f,
    .ki = 80'000.0f,
    .kd = 7'000.0f,
    .maxICumulative = 7000.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 10.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 2.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
    .antiSaturation = true,
};

#elif defined(TARGET_STANDARD_PHOBOS)
// tuned
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 40'000.0f,
    .ki = 1'000'000.0f,
    .kd = 4'000.0f,
    .maxICumulative = 4000.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 70.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
    .antiSaturation = true,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    .kp = 20'000.0f,
    .ki = 500'000.0f,
    .kd = 3'000.0f,
    .maxICumulative = 2'000.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 10.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 2.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
    .antiSaturation = true,
};
#else
#error "Attempted to include standard_turret_constants.hpp for nonstandard target."
#endif
}  // namespace chassis_rel
}  // namespace aruwsrc::control::turret

#endif  // STANDARD_TURRET_CONSTANTS_HPP_
