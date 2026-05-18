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
#include "aruwsrc/robot/engineer/wrist/wrist_setpoints_command.hpp"
#include "aruwsrc/robot/engineer/wrist/wrist_subsystem.hpp"

namespace aruwsrc::engineer
{
static constexpr tap::can::CanBus CAN_BUS_WRIST = tap::can::CanBus::CAN_BUS1;
static constexpr tap::motor::MotorId WRIST_MOTOR_1_ID = tap::motor::MotorId::MOTOR4;
static constexpr tap::motor::MotorId WRIST_MOTOR_2_ID = tap::motor::MotorId::MOTOR2;
static constexpr tap::motor::MotorId WRIST_THETA3_MOTOR_ID = tap::motor::MotorId::MOTOR3;
static constexpr tap::encoder::CanEncoderId WRIST_THETA1_ENCODER_ID =
    tap::encoder::CanEncoderId::ID1;  // todo: lamprey
static constexpr tap::encoder::CanEncoderId WRIST_THETA2_ENCODER_ID =
    tap::encoder::CanEncoderId::ID0;

static constexpr uint32_t WRIST_HOME_THETA1 = 0;
static constexpr uint32_t WRIST_HOME_THETA2 = 568;
static constexpr uint32_t WRIST_HOME_THETA3 = 0;

inline constexpr float WRIST_MOTOR_1_GEAR_RATIO = 10.0f / 28.0f;
inline constexpr float WRIST_MOTOR_2_GEAR_RATIO = 10.0f / 32.0f;
inline constexpr float WRIST_MOTOR_3_GEAR_RATIO = 12.0f / 30.0f;

static constexpr float WRIST_ROLL_PID_KS = 0.0;
static constexpr tap::algorithms::SmoothPidConfig WRIST_THETA1_PID_CONFIG{
    .kp = 4000.0f,
    .ki = 0.0f,
    .kd = 500.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C610,
};

static constexpr tap::algorithms::SmoothPidConfig WRIST_THETA2_PID_CONFIG{
    .kp = 4000.0f,
    .ki = 0.0f,
    .kd = 500.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C610,
};

static constexpr tap::algorithms::SmoothPidConfig WRIST_THETA3_PID_CONFIG{
    .kp = 3000.0f,
    .ki = 0.0f,
    .kd = 300.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C610,
};

static constexpr wrist::WristConfig WRIST_CONFIG{
    .theta1PidConfig = WRIST_THETA1_PID_CONFIG,
    .theta2PidConfig = WRIST_THETA2_PID_CONFIG,
    .theta3PidConfig = WRIST_THETA3_PID_CONFIG,
    .theta2Min = -M_PI_2,
    .theta2Max = M_PI_2,
    .ratio = 1.0f,
    .maxMotorDesiredOutput = static_cast<int32_t>(tap::motor::DjiMotor::MAX_OUTPUT_C610 * 0.6f),
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

static constexpr float WRIST_THETA_1_SCALING_FACTOR = 0.01f;
static constexpr float WRIST_THETA_2_SCALING_FACTOR = 0.01f;
static constexpr float WRIST_THETA_3_SCALING_FACTOR = 0.01f;

static constexpr float WRIST_ROLL_CLICK_VELOCITY = 0.5f;

static constexpr wrist::Setpoint WRIST_IN_SETPOINT{
    .theta1 = 0,
    .theta2 = 0,
    .theta3 = 0,
    .epsilonTheta1 = 0.1f,
    .epsilonTheta2 = 0.1f,
    .epsilonTheta3 = 0.1f,
};
static constexpr wrist::Setpoint WRIST_TOP_SETPOINT{
    .theta1 = 0,
    .theta2 = 0,
    .theta3 = 0,
    .epsilonTheta1 = 0.1f,
    .epsilonTheta2 = 0.1f,
    .epsilonTheta3 = 0.1f,
};
static constexpr wrist::Setpoint WRIST_BOTTOM_SETPOINT{
    .theta1 = 0,
    .theta2 = 0,
    .theta3 = 0,
    .epsilonTheta1 = 0.1f,
    .epsilonTheta2 = 0.1f,
    .epsilonTheta3 = 0.1f,
};
static constexpr wrist::Setpoint WRIST_OUT_SETPOINT{
    .theta1 = 0,
    .theta2 = 0,
    .theta3 = 0,
    .epsilonTheta1 = 0.1f,
    .epsilonTheta2 = 0.1f,
    .epsilonTheta3 = 0.1f,
};

}  // namespace aruwsrc::engineer
#endif  // ENGINEER_WRIST_CONSTANTS_HPP_   `