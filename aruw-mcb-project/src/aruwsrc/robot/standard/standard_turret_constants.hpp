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

using tap::motor::DjiMotor;

namespace aruwsrc::control::turret
{
static constexpr uint8_t NUM_TURRETS = 1;

static constexpr float USER_YAW_INPUT_SCALAR = 0.02f;
static constexpr float USER_PITCH_INPUT_SCALAR = 0.02f;

static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;
static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR6;
static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR5;

#if defined(TARGET_STANDARD_SPIDER)
static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = M_PI_2,
    .startEncoderValue = 656,
    .minAngle = 0,
    .maxAngle = M_PI,
    .limitMotorAngles = false,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = M_PI_2,
    .startEncoderValue = 7500,
    .minAngle = modm::toRadian(50),
    .maxAngle = modm::toRadian(108),
    .limitMotorAngles = true,
};
#elif defined(TARGET_STANDARD_ORION)
static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = M_PI_2,
    .startEncoderValue = 1365,
    .minAngle = 0,
    .maxAngle = M_PI,
    .limitMotorAngles = false,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = M_PI_2,
    .startEncoderValue = 4705,
    .minAngle = modm::toRadian(48),
    .maxAngle = modm::toRadian(115),
    .limitMotorAngles = true,
};
#elif defined(TARGET_STANDARD_CYGNUS)
static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = M_PI_2,
    .startEncoderValue = 2693,
    .minAngle = 0,
    .maxAngle = M_PI,
    .limitMotorAngles = false,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = M_PI_2,
    .startEncoderValue = 3393,
    .minAngle = modm::toRadian(48),
    .maxAngle = modm::toRadian(115),
    .limitMotorAngles = true,
};
#else
#error "Attempted to include standard_turret_constants.hpp for nonstandard target."
#endif

#if defined(TARGET_STANDARD_ORION) || defined(TARGET_STANDARD_CYGNUS)
// Actual CAD value is 55.76, decreased for balls in hopper
static constexpr float TURRET_CG_X = 35.76;
static constexpr float TURRET_CG_Z = 52.25;
static constexpr float GRAVITY_COMPENSATION_SCALAR = -11'500;
#elif defined(TARGET_STANDARD_SPIDER)
static constexpr float TURRET_CG_X = 30.17;
static constexpr float TURRET_CG_Z = 34.02;
static constexpr float GRAVITY_COMPENSATION_SCALAR = 7'000;
#else
#error "Attempted to include standard_turret_constants.hpp for nonstandard target."
#endif

namespace world_rel_turret_imu
{
#if defined(TARGET_STANDARD_ORION)
static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_CONFIG = {
    .kp = 10.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 60.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_AUTO_AIM_CONFIG = {
    .kp = 25.0f,
    .ki = 0.15f,
    .kd = 0.0f,
    .maxICumulative = 0.6f,
    .maxOutput = 60.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_VEL_PID_CONFIG = {
    .kp = 28'000.0f,
    .ki = 100.0f,
    .kd = 0.0f,
    .maxICumulative = 2'000.0f,
    .maxOutput = DjiMotor::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_CONFIG = {
    .kp = 20.5f,
    .ki = 0.0f,
    .kd = 0.2f,
    .maxICumulative = 0.5f,
    .maxOutput = 30.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_AUTO_AIM_CONFIG = {
    .kp = 30.0f,
    .ki = 0.6f,
    .kd = 0.2f,
    .maxICumulative = 0.5f,
    .maxOutput = 30.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_VEL_PID_CONFIG = {
    .kp = 18'500.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotor::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
#elif defined(TARGET_STANDARD_CYGNUS)
static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_CONFIG = {
    .kp = 10.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 60.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_AUTO_AIM_CONFIG = {
    .kp = 20.0f,
    .ki = 0.15f,
    .kd = 0.0f,
    .maxICumulative = 0.6f,
    .maxOutput = 60.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_VEL_PID_CONFIG = {
    .kp = 28'000.0f,
    .ki = 100.0f,
    .kd = 0.0f,
    .maxICumulative = 2'000.0f,
    .maxOutput = DjiMotor::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_CONFIG = {
    .kp = 20.0f,
    .ki = 0.0f,
    .kd = 0.2f,
    .maxICumulative = 0.5f,
    .maxOutput = 30.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_AUTO_AIM_CONFIG = {
    .kp = 30.0f,
    .ki = 0.6f,
    .kd = 0.2f,
    .maxICumulative = 0.5f,
    .maxOutput = 30.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_VEL_PID_CONFIG = {
    .kp = 18'500.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotor::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
#elif defined(TARGET_STANDARD_SPIDER)
static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_CONFIG = {
    .kp = 22.0f,
    .ki = 0.0f,
    .kd = 0.3f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotor::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_AUTO_AIM_CONFIG = {
    .kp = 40.0f,
    .ki = 0.0f,
    .kd = 0.3f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotor::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_VEL_PID_CONFIG = {
    .kp = 20'000.0f,
    .ki = 100.0f,
    .kd = 0.0f,
    .maxICumulative = 2'000.0f,
    .maxOutput = DjiMotor::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_CONFIG = {
    .kp = 22.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.5f,
    .maxOutput = DjiMotor::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_AUTO_AIM_CONFIG = {
    .kp = 45.0f,
    .ki = 0.6f,
    .kd = 1.0f,
    .maxICumulative = 0.5f,
    .maxOutput = DjiMotor::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_VEL_PID_CONFIG = {
    .kp = 16'000.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotor::MAX_OUTPUT_GM6020,
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
    .kp = 200'535.2f,
    .ki = 0.0f,
    .kd = 10'886.2f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotor::MAX_OUTPUT_GM6020,
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
#if defined(TARGET_STANDARD_ORION)
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 229'183.1f,
    .ki = 0.0f,
    .kd = 10'886.2f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotor::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 70.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    .kp = 229'183.1f,
    .ki = 0.0f,
    .kd = 7'448.5f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotor::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 10.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 2.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

#elif defined(TARGET_STANDARD_CYGNUS)
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 229'183.1f,
    .ki = 0.0f,
    .kd = 10'886.2f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotor::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 70.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    .kp = 100'000.0f,
    .ki = 10.0f,
    .kd = 7'000.0f,
    .maxICumulative = 5000.0f,
    .maxOutput = DjiMotor::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 10.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 2.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

#elif defined(TARGET_STANDARD_SPIDER)
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 229'183.1f,
    .ki = 0.0f,
    .kd = 10'886.2f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotor::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 70.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    .kp = 229'183.1f,
    .ki = 0.0f,
    .kd = 7'448.5f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotor::MAX_OUTPUT_GM6020,
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
