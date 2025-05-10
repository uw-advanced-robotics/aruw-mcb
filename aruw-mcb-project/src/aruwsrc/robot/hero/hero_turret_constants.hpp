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

static constexpr tap::can::CanBus CAN_BUS_YAW_MOTOR = tap::can::CanBus::CAN_BUS1;
static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR5;

static constexpr tap::can::CanBus CAN_BUS_PITCH_MOTOR = tap::can::CanBus::CAN_BUS1;
static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR6;

static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 1871,
    .minAngle = 0,     ///< Doesn't matter since yaw not limited
    .maxAngle = M_PI,  ///< Doesn't matter since yaw not limited
    .limitMotorAngles = false,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 6855,
    .minAngle = modm::toRadian(-20),
    .maxAngle = modm::toRadian(30),
    .limitMotorAngles = true,
};

static constexpr float TURRET_CG_X = -40.16;
static constexpr float TURRET_CG_Z = 16.25;
static constexpr float GRAVITY_COMPENSATION_SCALAR = 13'000.0f;

namespace world_rel_turret_imu
{
static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_CONFIG = {
    // .kp = 10.0f,
    // .ki = 0.0f,
    // .kd = 0.0f,
    // .maxICumulative = 0.0f,
    // .maxOutput = 1.0,
    // .tQDerivativeKalman = 1.0f,
    // .tRDerivativeKalman = 0.0f,
    // .tQProportionalKalman = 1.0f,
    // .tRProportionalKalman = 5.0f,
    // .errDeadzone = 0.0f,
    // .errorDerivativeFloor = 0.0f,
    .kp = 12.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 2.5,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 5.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_AUTO_AIM_CONFIG = {
    // .kp = 25.5f,
    // .ki = 0.0f,
    // .kd = 0.03f,
    // .maxICumulative = 0.0f,
    // .maxOutput = 1.0f,
    // .tQDerivativeKalman = 1.0f,
    // .tRDerivativeKalman = 0.0f,
    // .tQProportionalKalman = 1.0f,
    // .tRProportionalKalman = 0.0f,
    // .errDeadzone = 0.0f,
    // .errorDerivativeFloor = 0.0f,
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = M_TWOPI,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_VEL_PID_CONFIG = {
    // .kp = 2500.0f,
    // .ki = 0.0f,
    // .kd = 0.0f,
    // .maxICumulative = 2'000.0f,
    // .maxOutput = 1.0f,
    // .tQDerivativeKalman = 1.0f,
    // .tRDerivativeKalman = 0.0f,
    // .tQProportionalKalman = 1.0f,
    // .tRProportionalKalman = 0.5f,
    // .errDeadzone = 0.0f,
    // .errorDerivativeFloor = 0.0f,
    .kp = 9'000.0f,
    .ki = 30.0f,
    .kd = 0.0f,
    .maxICumulative = 1'800.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C620,
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
    .maxICumulative = 0.0f,
    .maxOutput = 30.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_AUTO_AIM_CONFIG = {
    .kp = 22.0f,
    .ki = 0.0f,
    .kd = 0.6f,
    .maxICumulative = 0.0f,
    .maxOutput = 30.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_VEL_PID_CONFIG = {
    .kp = 17'000.0f,
    .ki = 400.0f,
    .kd = 0.0f,
    .maxICumulative = 5'000.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020,
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
    // .kp = 2500.0f,
    // .ki = 0.0f,
    // .kd = 0.0f,
    // .maxICumulative = 0.0f,
    // .maxOutput = 1.0f,
    // .tQDerivativeKalman = 1.0f,
    // .tRDerivativeKalman = 40.0f,
    // .tQProportionalKalman = 1.0f,
    // .tRProportionalKalman = 0.0f,
    // .errDeadzone = 0.0f,
    // .errorDerivativeFloor = 0.0f,
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020,
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
    // .kp = 2500.0f,
    // .ki = 0.0f,
    // .kd = 0.0f,
    // .maxICumulative = 0.0f,
    // .maxOutput = 1.0f,
    // .tQDerivativeKalman = 1.0f,
    // .tRDerivativeKalman = 40.0f,
    // .tQProportionalKalman = 1.0f,
    // .tRProportionalKalman = 0.0f,
    // .errDeadzone = 0.0f,
    // .errorDerivativeFloor = 0.0f,
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 0.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 0.0f,
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
}  // namespace chassis_rel

}  // namespace aruwsrc::control::turret

#endif  // HERO_TURRET_CONSTANTS_HPP_
