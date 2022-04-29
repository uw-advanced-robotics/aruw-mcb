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
#define SENTINEL_2022_TURRET_CONscoSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/dji_motor.hpp"

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

namespace turret1
{
static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = M_PI_2,
    .startEncoderValue = 5455,
    .minAngle = -M_PI_2,
    .maxAngle = M_PI,
    .limitMotorAngles = true,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = M_PI_2,
    .startEncoderValue = 4096,
    .minAngle = modm::toRadian(80),
    .maxAngle = modm::toRadian(150),
    .limitMotorAngles = true,
};
}  // namespace turret1

namespace turret2
{
static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS2;

static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = M_PI_2,
    .startEncoderValue = 7370,
    .minAngle = -5 * M_PI_2 / 4,
    .maxAngle = 3 * M_PI / 4,
    .limitMotorAngles = true,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = M_PI_2,
    .startEncoderValue = 0,
    .minAngle = modm::toRadian(80),
    .maxAngle = modm::toRadian(150),
    .limitMotorAngles = true,
};
}  // namespace turret2

static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR6;
static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR5;

static constexpr float TURRET_CG_X = 0;
static constexpr float TURRET_CG_Z = 0;
static constexpr float GRAVITY_COMPENSATION_SCALAR = 1.0f;

namespace chassis_rel
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 180'000.0f,
    .ki = 0.1f,
    .kd = 9'000.0,
    .maxICumulative = 0.0f,
    .maxOutput = 30'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 10.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    .kp = 194'805.7f,
    .ki = 0.0f,
    .kd = 5'729.6f,
    .maxICumulative = 0.0f,
    .maxOutput = 30'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 20.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};
}  // namespace chassis_rel
}  // namespace  aruwsrc::control::turret

#endif  // SENTINEL_TURRET_CONSTANTS_HPP_
