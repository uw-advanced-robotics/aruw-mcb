/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef ENGINEER_WRIST_CONSTANTS_HPP_
#define ENGINEER_WRIST_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/sensors/encoder/can_encoder/can_encoder.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/robot/engineer/wrist/wrist_subsystem.hpp"

namespace aruwsrc::engineer
{
static constexpr tap::can::CanBus CAN_BUS_WRIST = tap::can::CanBus::CAN_BUS1;
static constexpr tap::motor::MotorId WRIST_LEFT_MOTOR_ID = tap::motor::MotorId::MOTOR4;
static constexpr tap::motor::MotorId WRIST_RIGHT_MOTOR_ID = tap::motor::MotorId::MOTOR5;
static constexpr tap::motor::MotorId WRIST_ROLL_MOTOR_ID = tap::motor::MotorId::MOTOR6;
static constexpr tap::encoder::CanEncoderId WRIST_PITCH_ENCODER_ID =
    tap::encoder::CanEncoderId::ID0;
static constexpr tap::encoder::CanEncoderId WRIST_YAW_ENCODER_ID = tap::encoder::CanEncoderId::ID1;

static constexpr wrist::WristConfig WRIST_CONFIG{
    .pitchPidConfig =
        {
            .kp = 8000.0f,
            .ki = 0.0f,
            .kd = 700.0f,
            .maxICumulative = 1000.0f,
            .maxOutput = 5000.0f,
            .tQDerivativeKalman = 1.0f,
            .tRDerivativeKalman = 30.0f,
            .tQProportionalKalman = 1.0f,
            .tRProportionalKalman = 0.0f,
            .errDeadzone = 0.0f,
            .errorDerivativeFloor = 0.025f,
        },
    .yawPidConfig =
        {
            .kp = 12000.0f,
            .ki = 0.0f,
            .kd = 1000.0f,
            .maxICumulative = 500.0f,
            .maxOutput = 5500.0f,
            .tQDerivativeKalman = 1.0f,
            .tRDerivativeKalman = 30.0f,
            .tQProportionalKalman = 1.0f,
            .tRProportionalKalman = 0.0f,
            .errDeadzone = 0.0f,
            .errorDerivativeFloor = 0.0,
        },
    .minPitch = 0.0f,
    .maxPitch = M_PI_2,
    .minYaw = -M_PI_2,
    .maxYaw = M_PI,
    .ratio = 30.0f / 40.0f,
    .epsilon = 0.1f,
    .maxMotorDesiredOutput = 3000,
};

static constexpr uint32_t WRIST_HOME_PITCH = 2454;
static constexpr uint32_t WRIST_HOME_YAW = 1961;

static constexpr float WRIST_ROLL_PID_KS = 0.0;
static constexpr tap::algorithms::SmoothPidConfig WRIST_ROLL_PID_CONFIG{
    .kp = 200.0f,
    .ki = 0.0f,
    .kd = 15.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 3000.0f,
};

static constexpr float WRIST_ROLL_SCALING_FACTOR = 0.25f;
static constexpr float WRIST_PITCH_SCALING_FACTOR = 0.01f;
static constexpr float WRIST_YAW_SCALING_FACTOR = 0.01f;

static constexpr float WRIST_TOP_SETPOINT_PITCH = 1.5f;
static constexpr float WRIST_TOP_SETPOINT_YAW = M_PI;

static constexpr float WRIST_BOTTOM_SETPOINT_PITCH = 1.5f;
static constexpr float WRIST_BOTTOM_SETPOINT_YAW = 0.0f;

static constexpr float WRIST_IN_SETPOINT_PITCH = 0.0f;
static constexpr float WRIST_IN_SETPOINT_YAW = M_PI;

static constexpr float WRIST_OUT_SETPOINT_PITCH = 0.0f;
static constexpr float WRIST_OUT_SETPOINT_YAW = 0.0f;

}  // namespace aruwsrc::engineer
#endif  // ENGINEER_WRIST_CONSTANTS_HPP_   `