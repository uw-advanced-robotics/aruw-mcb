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

#include "aruwsrc/robot/engineer/wrist/wrist_setpoints_command.hpp"

namespace aruwsrc::engineer
{
static constexpr tap::can::CanBus CAN_BUS_WRIST = tap::can::CanBus::CAN_BUS1;
static constexpr tap::motor::MotorId WRIST_LEFT_MOTOR_ID = tap::motor::MotorId::MOTOR4;
static constexpr tap::motor::MotorId WRIST_RIGHT_MOTOR_ID = tap::motor::MotorId::MOTOR5;
static constexpr tap::motor::MotorId WRIST_ROLL_MOTOR_ID = tap::motor::MotorId::MOTOR6;
static constexpr tap::encoder::CanEncoderId WRIST_PITCH_ENCODER_ID =
    tap::encoder::CanEncoderId::ID0;
static constexpr tap::encoder::CanEncoderId WRIST_YAW_ENCODER_ID = tap::encoder::CanEncoderId::ID1;

static constexpr float WRIST_PITCH_PID_KP = 2500.0f;
static constexpr float WRIST_PITCH_PID_KI = 0.1f;
static constexpr float WRIST_PITCH_PID_KD = 30.0f;
static constexpr float WRIST_PITCH_PID_MAX_ERROR_SUM = 1000.0f;
static constexpr float WRIST_PITCH_PID_KS = 0.0;
static constexpr float WRIST_PITCH_MAX_OUTPUT = 3000.0f;

// units of radians
static constexpr float WRIST_MIN_PITCH = -M_PI_2;
static constexpr float WRIST_MAX_PITCH = M_PI * 3.0f / 2.0f;  // todo
static constexpr float WRIST_HOME_PITCH = 2.46817517f;        // todo

static constexpr float WRIST_YAW_PID_KP = 2000.0f;
static constexpr float WRIST_YAW_PID_KI = 0.0f;
static constexpr float WRIST_YAW_PID_KD = 60.0f;
static constexpr float WRIST_YAW_PID_MAX_ERROR_SUM = 0.0f;
static constexpr float WRIST_YAW_PID_KS = 0.0;
static constexpr float WRIST_YAW_MAX_OUTPUT = 1000.0f;

// units of radians
static constexpr float WRIST_MIN_YAW = -M_PI * 2;
static constexpr float WRIST_MAX_YAW = M_PI * 2;  // todo
static constexpr float WRIST_HOME_YAW = 1.23485458f;

static constexpr float WRIST_ROLL_PID_KP = 200.0f;
static constexpr float WRIST_ROLL_PID_KI = 0.0f;
static constexpr float WRIST_ROLL_PID_KD = 15.0f;
static constexpr float WRIST_ROLL_PID_MAX_ERROR_SUM = 0.0f;
static constexpr float WRIST_ROLL_PID_KS = 0.0;
static constexpr float WRIST_ROLL_MAX_OUTPUT = 3000.0f;

static constexpr float WRIST_RATIO = 1.0f;

static constexpr tap::algorithms::SmoothPidConfig WRIST_PITCH_CONFIG(
    WRIST_PITCH_PID_KP,
    WRIST_PITCH_PID_KI,
    WRIST_PITCH_PID_KD,
    WRIST_PITCH_PID_MAX_ERROR_SUM,
    WRIST_PITCH_MAX_OUTPUT);

static constexpr tap::algorithms::SmoothPidConfig WRIST_YAW_CONFIG(
    WRIST_YAW_PID_KP,
    WRIST_YAW_PID_KI,
    WRIST_YAW_PID_KD,
    WRIST_YAW_PID_MAX_ERROR_SUM,
    WRIST_YAW_MAX_OUTPUT);

static constexpr tap::algorithms::SmoothPidConfig WRIST_ROLL_CONFIG(
    WRIST_ROLL_PID_KP,
    WRIST_ROLL_PID_KI,
    WRIST_ROLL_PID_KD,
    WRIST_ROLL_PID_MAX_ERROR_SUM,
    WRIST_ROLL_MAX_OUTPUT);

static constexpr float WRIST_ROLL_SCALING_FACTOR = 0.25f;
static constexpr float WRIST_PITCH_SCALING_FACTOR = 0.01f;
static constexpr float WRIST_YAW_SCALING_FACTOR = 0.01f;

static constexpr wrist::Setpoint WRIST_IN_SETPOINT{
    0,
    0,
    0.1f,  // epsilonPitch
    0.1f   // epsilonYaw
};
static constexpr wrist::Setpoint WRIST_TOP_SETPOINT{
    1.5f,
    0,
    0.1f,  // epsilonPitch
    0.1f   // epsilonYaw
};
static constexpr wrist::Setpoint WRIST_BOTTOM_SETPOINT{
    1.5f,
    M_PI,
    0.1f,  // epsilonPitch
    0.1f   // epsilonYaw
};
static constexpr wrist::Setpoint WRIST_OUT_SETPOINT{
    0,
    M_PI,
    0.1f,  // epsilonPitch
    0.1f   // epsilonYaw
};

}  // namespace aruwsrc::engineer
#endif  // ENGINEER_WRIST_CONSTANTS_HPP_   `