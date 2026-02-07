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

#include "aruwsrc/control/joint/joint_subsystem.hpp"
#include "wrist/wrist_setpoints_command.hpp"
#include "wrist/wrist_subsystem.hpp"

namespace aruwsrc::engineer
{
static constexpr tap::can::CanBus CAN_BUS_WRIST = tap::can::CanBus::CAN_BUS1;
static constexpr tap::motor::MotorId WRIST_LEFT_MOTOR_ID = tap::motor::MotorId::MOTOR4;
static constexpr tap::motor::MotorId WRIST_RIGHT_MOTOR_ID = tap::motor::MotorId::MOTOR5;
static constexpr tap::motor::MotorId WRIST_THETA3_MOTOR_ID = tap::motor::MotorId::MOTOR6;
static constexpr tap::encoder::CanEncoderId WRIST_THETA1_ENCODER_ID =
    tap::encoder::CanEncoderId::ID0;
static constexpr tap::encoder::CanEncoderId WRIST_THETA2_ENCODER_ID =
    tap::encoder::CanEncoderId::ID1;
static constexpr tap::encoder::CanEncoderId WRIST_THETA3_ENCODER_ID =
    tap::encoder::CanEncoderId::ID2;


static constexpr uint32_t WRIST_HOME_THETA1 = 0;
static constexpr uint32_t WRIST_HOME_THETA2 = 0;
static constexpr uint32_t WRIST_HOME_THETA3 = 0;



static constexpr float WRIST_ROLL_PID_KS = 0.0;
static constexpr tap::algorithms::SmoothPidConfig WRIST_THETA1_PID_CONFIG{
    .kp = 0,
    .ki = 0.0f,
    .kd = 0,
    .maxICumulative = 0.0f,
    .maxOutput = 3000.0f,
};

static constexpr tap::algorithms::SmoothPidConfig WRIST_THETA2_PID_CONFIG{
    .kp = 0,
    .ki = 0.0f,
    .kd = 0,
    .maxICumulative = 0.0f,
    .maxOutput = 3000.0f,
};

static constexpr tap::algorithms::SmoothPidConfig WRIST_THETA3_PID_CONFIG{
    .kp = 0,
    .ki = 0.0f,
    .kd = 0,
    .maxICumulative = 0.0f,
    .maxOutput = 3000.0f,
};

static constexpr wrist::WristConfig WRIST_CONFIG{
    .theta1PidConfig = WRIST_THETA1_PID_CONFIG,
    .theta2PidConfig = WRIST_THETA2_PID_CONFIG,
    .theta3PidConfig = WRIST_THETA3_PID_CONFIG,
    .ratio = 30.0f / 40.0f,
    .maxMotorDesiredOutput = 5500,
};

static constexpr tap::algorithms::SmoothPidConfig WRIST_ROLL_PID_CONFIG{
    .kp = 200.0f,
    .ki = 0.0f,
    .kd = 15.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 3000.0f,
};

static constexpr aruwsrc::control::joint::JointSubsystem::Config WRIST_ROLL_CONFIG{
    .epsilon = 1e-2,
    .posPidConfig = WRIST_ROLL_PID_CONFIG,
    .maxOutput = WRIST_ROLL_PID_CONFIG.maxOutput,
};

static constexpr float WRIST_ROLL_SCALING_FACTOR = 0.01f;
static constexpr float WRIST_PITCH_SCALING_FACTOR = 0.01f;
static constexpr float WRIST_YAW_SCALING_FACTOR = 0.01f;

static constexpr float WRIST_ROLL_CLICK_VELOCITY = 0.5f;

static constexpr wrist::Setpoint WRIST_IN_SETPOINT{
    .pitch = 0,
    .yaw = 0,
    .epsilonPitch = 0.1f,
    .epsilonYaw = 0.1f,
};
static constexpr wrist::Setpoint WRIST_TOP_SETPOINT{
    .pitch = 0,
    .yaw = 0,
    .epsilonPitch = 0.1f,
    .epsilonYaw = 0.1f,
};
static constexpr wrist::Setpoint WRIST_BOTTOM_SETPOINT{
    .pitch = 0,
    .yaw = 0,
    .epsilonPitch = 0.1f,
    .epsilonYaw = 0.1f,
};
static constexpr wrist::Setpoint WRIST_OUT_SETPOINT{
    .pitch = 0,
    .yaw = 0,
    .epsilonPitch = 0.1f,
    .epsilonYaw = 0.1f,
};

}  // namespace aruwsrc::engineer
#endif  // ENGINEER_WRIST_CONSTANTS_HPP_   `