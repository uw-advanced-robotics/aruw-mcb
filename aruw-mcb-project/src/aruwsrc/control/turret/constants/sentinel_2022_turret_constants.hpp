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

#ifndef SENTINEL_2022_TURRET_CONSTANTS_HPP_
#define SENTINEL_2022_TURRET_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/dji_motor.hpp"

#include "../cv/turret_scan_command.hpp"
#include "../turret_motor_config.hpp"
#include "modm/math/geometry/angle.hpp"

// Do not include this file directly: use turret_constants.hpp instead.
#ifndef TURRET_CONSTANTS_HPP_
#error "Do not include this file directly! Use turret_controller_constants.hpp instead."
#endif

namespace aruwsrc::control::turret
{
static constexpr uint8_t NUM_TURRETS = 2;

static constexpr float USER_YAW_INPUT_SCALAR = 0.02f;
static constexpr float USER_PITCH_INPUT_SCALAR = 0.02f;

namespace turret0
{
static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS2;

static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 7655,
    .minAngle = -M_PI - modm::toRadian(60),
    .maxAngle = modm::toRadian(30),
    .limitMotorAngles = true,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 0,
    .minAngle = modm::toRadian(-10),
    .maxAngle = modm::toRadian(60),
    .limitMotorAngles = true,
};

static constexpr cv::TurretScanCommand::Config TURRET_SCAN_CONFIG = {
    .scanLowPassAlpha = 0.007f,
    .aimLostNumCounts = 500,
    .pitchScanConfig =
        {
            .lowerBound = PITCH_MOTOR_CONFIG.minAngle,
            .upperBound = PITCH_MOTOR_CONFIG.maxAngle,
            .delta = modm::toRadian(0.2f),
        },
    .yawScanConfig =
        {
            .lowerBound = YAW_MOTOR_CONFIG.minAngle + modm::toRadian(1),
            .upperBound = YAW_MOTOR_CONFIG.maxAngle - modm::toRadian(1),
            .delta = modm::toRadian(0.2f),
        },
};
}  // namespace turret0

namespace turret1
{
static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 1050,
    .minAngle = -M_PI_2,
    .maxAngle = M_PI + modm::toRadian(30),
    .limitMotorAngles = true,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 1365,
    .minAngle = modm::toRadian(-15),
    .maxAngle = modm::toRadian(60),
    .limitMotorAngles = true,
};

static constexpr cv::TurretScanCommand::Config TURRET_SCAN_CONFIG = {
    .scanLowPassAlpha = 0.007f,
    .aimLostNumCounts = 500,
    .pitchScanConfig =
        {
            .lowerBound = PITCH_MOTOR_CONFIG.minAngle,
            .upperBound = PITCH_MOTOR_CONFIG.maxAngle,
            .delta = modm::toRadian(0.2f),
        },
    .yawScanConfig =
        {
            .lowerBound = YAW_MOTOR_CONFIG.minAngle + modm::toRadian(1),
            .upperBound = YAW_MOTOR_CONFIG.maxAngle - modm::toRadian(1),
            .delta = modm::toRadian(0.2f),
        },
};
}  // namespace turret1

static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR6;
static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR5;

static constexpr float TURRET_CG_X = -48.14f;
static constexpr float TURRET_CG_Z = 9.45f;
static constexpr float GRAVITY_COMPENSATION_SCALAR = 7'200.0f;

namespace chassis_rel
{
namespace turret0
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 70'000.0f,
    .ki = 0.0f,
    .kd = 3'000.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 30'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 40.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    .kp = 180'000.0f,
    .ki = 1'000.0f,
    .kd = 5'000.0f,
    .maxICumulative = 3'000.0f,
    .maxOutput = 30'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 20.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};
}  // namespace turret0

namespace turret1
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 70'000.0f,
    .ki = 0.0f,
    .kd = 3'000.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 30'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 40.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    .kp = 180'000.0f,
    .ki = 1'000.0f,
    .kd = 5'000.0f,
    .maxICumulative = 3'000.0f,
    .maxOutput = 30'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 20.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};
}  // namespace turret1
}  // namespace chassis_rel

namespace world_rel_turret_imu
{
namespace turret0
{
static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_CONFIG = {
    .kp = 18.0f,
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

static constexpr tap::algorithms::SmoothPidConfig YAW_VEL_PID_CONFIG = {
    .kp = 40'000.0f,
    .ki = 286.5f,
    .kd = 0.0f,
    .maxICumulative = 2'000.0f,
    .maxOutput = 30'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
}  // namespace turret0

namespace turret1
{
static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_CONFIG = {
    .kp = 18.0f,
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

static constexpr tap::algorithms::SmoothPidConfig YAW_VEL_PID_CONFIG = {
    .kp = 40'000.0f,
    .ki = 286.5f,
    .kd = 0.0f,
    .maxICumulative = 2'000.0f,
    .maxOutput = 30'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
}  // namespace turret1
}  // namespace world_rel_turret_imu
}  // namespace  aruwsrc::control::turret

#endif  // SENTINEL_2022_TURRET_CONSTANTS_HPP_
