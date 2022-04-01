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

#ifndef HERO_TURRET_CONTROLLER_CONSTANTS_HPP_
#define HERO_TURRET_CONTROLLER_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/dji_motor.hpp"

namespace aruwsrc::control::turret
{
static constexpr tap::can::CanBus CAN_BUS_YAW_MOTORS = tap::can::CanBus::CAN_BUS2;
static constexpr tap::can::CanBus CAN_BUS_PITCH_MOTOR = tap::can::CanBus::CAN_BUS1;
static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR7;
static constexpr tap::motor::MotorId YAW_FRONT_MOTOR_ID = tap::motor::MOTOR5;
static constexpr tap::motor::MotorId YAW_BACK_MOTOR_ID = tap::motor::MOTOR6;

static constexpr float YAW_START_ANGLE = 90.0f;
static constexpr float PITCH_START_ANGLE = 90.0f;
static constexpr float YAW_MIN_ANGLE = YAW_START_ANGLE - 70.0f;
static constexpr float YAW_MAX_ANGLE = YAW_START_ANGLE + 70.0f;
static constexpr float PITCH_MIN_ANGLE = 70.0f;
static constexpr float PITCH_MAX_ANGLE = 115.0f;

static constexpr uint16_t YAW_START_ENCODER_POSITION = 4872;
static constexpr uint16_t PITCH_START_ENCODER_POSITION = 3900;

static constexpr float TURRET_CG_X = 1;
static constexpr float TURRET_CG_Z = -0.2;
static constexpr float GRAVITY_COMPENSATION_SCALAR = 3500.0f;

namespace world_rel_turret_imu
{
static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_CONFIG = {
    .kp = 13.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 7000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_VEL_PID_CONFIG = {
    .kp = 700.0f,
    .ki = 5.0f,
    .kd = 0.0f,
    .maxICumulative = 2'000.0f,
    .maxOutput = 30000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
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
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_VEL_PID_CONFIG = {
    .kp = 750.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 30000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
};
}  // namespace world_rel_turret_imu

namespace world_rel_chassis_imu
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 2500.0f,
    .ki = 0.0f,
    .kd = 150.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 30000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 40.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};
}  // namespace world_rel_chassis_imu

namespace chassis_rel
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 2500.0f,
    .ki = 0.0f,
    .kd = 150.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 30000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 40.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    .kp = 4000.0f,
    .ki = 0.0f,
    .kd = 130.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 30000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 20.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 2.0f,
    .errDeadzone = 0.0f,
};
}  // namespace chassis_rel
}  // namespace aruwsrc::control::turret

#endif  // HERO_TURRET_CONTROLLER_CONSTANTS_HPP_
