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

static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;
#if defined(TARGET_STANDARD_NULL)
static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR6;
static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR5;
#else
static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR5;
static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR6;
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

#elif defined(TARGET_STANDARD_VOID)
static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 8184,
    .minAngle = 0,
    .maxAngle = M_PI,
    .limitMotorAngles = false,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 5191,
    .minAngle = modm::toRadian(-18),
    .maxAngle = modm::toRadian(28),
    .limitMotorAngles = true,
};
#else
#error "Attempted to include standard_turret_constants.hpp for nonstandard target."
#endif

#if defined(TARGET_STANDARD_NULL)
// Actual CAD value is 55.76, decreased for balls in hopper
static constexpr float TURRET_CG_X = 33.83;
static constexpr float TURRET_CG_Z = 26.68;
static constexpr float GRAVITY_COMPENSATION_SCALAR = -5'000;

#elif defined(TARGET_STANDARD_VOID)
static constexpr float TURRET_CG_X = 33.83;
static constexpr float TURRET_CG_Z = 26.68;
static constexpr float GRAVITY_COMPENSATION_SCALAR = -5'000;
#else
#error "Attempted to include standard_turret_constants.hpp for nonstandard target."
#endif

namespace world_rel_turret_imu
{
#if defined(TARGET_STANDARD_NULL)
static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_CONFIG = {
    .kp = 1050.0f,
    .ki = 0.0f,
    .kd = 0.3f,
    .maxICumulative = 0.0f,
    .maxOutput = 2000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_AUTO_AIM_CONFIG = {
    .kp = 1050.0f,
    .ki = 0.0f,
    .kd = 0.3f,
    .maxICumulative = 0.0f,
    .maxOutput = 2000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_VEL_PID_CONFIG = {
    .kp = 350.0f,
    .ki = 0.0f,
    .kd = 15.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_CONFIG = {
    .kp = 1500.0f,
    .ki = 0.1f,
    .kd = 0.3f,
    .maxICumulative = 200.0f,
    .maxOutput = 2000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_AUTO_AIM_CONFIG = {
    .kp = 1500.0f,
    .ki = 0.1f,
    .kd = 0.3f,
    .maxICumulative = 200.0f,
    .maxOutput = 2000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_VEL_PID_CONFIG = {
    .kp = 200.0f,
    .ki = 0.0f,
    .kd = 1.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

#elif defined(TARGET_STANDARD_VOID)
static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_CONFIG = {
    .kp = 1050.0f,
    .ki = 0.0f,
    .kd = 0.3f,
    .maxICumulative = 0.0f,
    .maxOutput = 2000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_AUTO_AIM_CONFIG = {
    .kp = 1050.0f,
    .ki = 0.0f,
    .kd = 0.3f,
    .maxICumulative = 0.0f,
    .maxOutput = 2000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_VEL_PID_CONFIG = {
    .kp = 350.0f,
    .ki = 0.0f,
    .kd = 15.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_CONFIG = {
    .kp = 1500.0f,
    .ki = 0.1f,
    .kd = 0.3f,
    .maxICumulative = 200.0f,
    .maxOutput = 2000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_AUTO_AIM_CONFIG = {
    .kp = 1500.0f,
    .ki = 0.1f,
    .kd = 0.3f,
    .maxICumulative = 200.0f,
    .maxOutput = 2000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_VEL_PID_CONFIG = {
    .kp = 200.0f,
    .ki = 0.0f,
    .kd = 1.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020,
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
    .kp = 25'000.0f,
    .ki = 0.0f,
    .kd = 5'000.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 40.0f,
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
    .ki = 0.0f,
    .kd = 5'000.2f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 70.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    .kp = 80'000.0f,
    .ki = 0.0f,
    .kd = 7'000.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 10.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 2.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

#elif defined(TARGET_STANDARD_VOID)
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 25'000.0f,
    .ki = 0.0f,
    .kd = 5'000.2f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 70.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    .kp = 80'000.0f,
    .ki = 0.0f,
    .kd = 7'000.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 10.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 2.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
#else
#error "Attempted to include standard_turret_constants.hpp for nonstandard target."
#endif
}  // namespace chassis_rel
}  // namespace aruwsrc::control::turret

#endif  // STANDARD_TURRET_CONSTANTS_HPP_
