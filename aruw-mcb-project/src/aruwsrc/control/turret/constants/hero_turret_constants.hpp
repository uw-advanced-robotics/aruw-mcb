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

#include "../turret_motor_config.hpp"
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

static constexpr tap::can::CanBus CAN_BUS_YAW_MOTORS = tap::can::CanBus::CAN_BUS2;
static constexpr tap::can::CanBus CAN_BUS_PITCH_MOTOR = tap::can::CanBus::CAN_BUS1;
static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR7;
static constexpr tap::motor::MotorId YAW_FRONT_MOTOR_ID = tap::motor::MOTOR5;
static constexpr tap::motor::MotorId YAW_BACK_MOTOR_ID = tap::motor::MOTOR6;

static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = M_PI_2,
    .startEncoderValue = 4872,
    .minAngle = 0,     ///< Doesn't matter since yaw not limited
    .maxAngle = M_PI,  ///< Doesn't matter since yaw not limited
    .limitMotorAngles = false,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = M_PI_2,
    .startEncoderValue = 3900,
    .minAngle = modm::toRadian(55),
    .maxAngle = modm::toRadian(115),
    .limitMotorAngles = true,
};

static constexpr float TURRET_CG_X = -22.23;
static constexpr float TURRET_CG_Z = 9.96;
static constexpr float GRAVITY_COMPENSATION_SCALAR = 3'500.0f;

namespace world_rel_turret_imu
{
static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_CONFIG = {
    .kp = 9.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 3'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.1f,
};

static tap::algorithms::FuzzyPDConfig YAW_FUZZY_POS_PD_CONFIG = {
    .maxError = M_PI,                      ///< 180 degrees physical max angle error
    .maxErrorDerivative = 2.0f * M_TWOPI,  ///< 2 rotations per second max speed of turret
    .fuzzyTable = tap::algorithms::FuzzyPDRuleTable(
        std::array<float, 3>({YAW_POS_PID_CONFIG.kp, YAW_POS_PID_CONFIG.kp, YAW_POS_PID_CONFIG.kp}),
        std::array<float, 3>({0.0f, 0.1f, 0.7f})),
};

static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_AUTO_AIM_CONFIG = {
    .kp = 20.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 3'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.1f,
};

static tap::algorithms::FuzzyPDConfig YAW_FUZZY_POS_PD_AUTO_AIM_CONFIG = {
    .maxError = M_PI,                      ///< 180 degrees physical max angle error
    .maxErrorDerivative = 2.0f * M_TWOPI,  ///< 2 rotations per second max speed of turret
    .fuzzyTable = tap::algorithms::FuzzyPDRuleTable(
        std::array<float, 3>(
            {YAW_POS_PID_AUTO_AIM_CONFIG.kp,
             YAW_POS_PID_AUTO_AIM_CONFIG.kp,
             YAW_POS_PID_AUTO_AIM_CONFIG.kp}),
        std::array<float, 3>({0.0f, 0.1f, 0.7f})),
};

static constexpr tap::algorithms::SmoothPidConfig YAW_VEL_PID_CONFIG = {
    .kp = 51'566.2f,
    .ki = 286.5f,
    .kd = 0.0f,
    .maxICumulative = 2'000.0f,
    .maxOutput = 30'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_CONFIG = {
    .kp = 22.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 10'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_VEL_PID_CONFIG = {
    .kp = 42'971.8f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 30'000.0f,
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
    .kp = 143'239.4f,
    .ki = 0.0f,
    .kd = 8594.4f,
    .maxICumulative = 0.0f,
    .maxOutput = 30'000.0f,
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
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 143'239.4f,
    .ki = 0.0f,
    .kd = 8594.4f,
    .maxICumulative = 0.0f,
    .maxOutput = 30'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 40.0f,
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
    .maxOutput = 30'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 20.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 2.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
}  // namespace chassis_rel
}  // namespace aruwsrc::control::turret

#endif  // HERO_TURRET_CONSTANTS_HPP_
