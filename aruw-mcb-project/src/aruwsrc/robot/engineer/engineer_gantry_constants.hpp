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

#ifndef ENGINEER_GANTRY_CONSTANTS_HPP_
#define ENGINEER_GANTRY_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/gpio/digital.hpp"
#include "tap/motor/dji_motor.hpp"
namespace aruwsrc::engineer
{
static constexpr tap::can::CanBus CAN_BUS_GANTRY = tap::can::CanBus::CAN_BUS1;

static constexpr tap::motor::MotorId GANTRY_LIFT_LEFT_MOTOR_ID = tap::motor::MotorId::MOTOR1;
static constexpr tap::motor::MotorId GANTRY_LIFT_RIGHT_MOTOR_ID = tap::motor::MotorId::MOTOR2;
static constexpr tap::motor::MotorId GANTRY_EXTENSION_MOTOR_ID = tap::motor::MotorId::MOTOR3;

static constexpr tap::gpio::Digital::InputPin GANTRY_LIFT_LIMIT_SWITCH_PIN =
    tap::gpio::Digital::InputPin::D;

static constexpr float GANTRY_LIFT_MOVE_SPEED = 0.6f;

static constexpr tap::algorithms::SmoothPidConfig GANTRY_LIFT_POS_PID_CONFIG = {
    .kp = 2000.0f,
    .ki = 30.0f,
    .kd = 250.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 6000.0f,
};

static constexpr tap::algorithms::SmoothPidConfig GANTRY_LIFT_ALIGN_PID_CONFIG = {
    .kp = 100.0f,
    .ki = 00.0f,
    .kd = 00.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 1000.0f,
};

static constexpr TriggerHomedJointSubsystem::Config GANTRY_LIFT_CONFIG{
    .super =  // JointSubsystem::Config
    {
        .super =  // LinearJointInterface::Config
        {
            .lowerBound = 5.0f,
            .upperBound = 320.0f,
            .epsilon = 1.0f,
            .maxSetpointIncrement = 0.7f,
        },
        .encoderRatio = 5 * 14 / M_TWOPI,  // 5mm per tooth, 14 teeth
        .posPidConfig = GANTRY_LIFT_POS_PID_CONFIG,
        .maxOutput = GANTRY_LIFT_POS_PID_CONFIG.maxOutput,
        .staticFeedforward = 0.0f,
    },
    .home = 0.0f,
    .homingSpeed = 10.0f,
    .homingReversed = false,
};

static constexpr tap::gpio::Digital::InputPin GANTRY_EXTENSION_LIMIT_SWITCH_PIN =
    tap::gpio::Digital::InputPin::T;

static constexpr float GANTRY_EXTENSION_MOVE_SPEED = 0.6f;

static constexpr tap::algorithms::SmoothPidConfig GANTRY_EXTENSION_PID_CONFIG = {
    .kp = 300.0f,
    .ki = 0.0f,
    .kd = 40.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 2000.0f,
};

static constexpr TriggerHomedJointSubsystem::Config GANTRY_EXTENSION_CONFIG{
    .super =  // JointSubsystem::Config
    {
        .super =  // LinearJointInterface::Config
        {
            .lowerBound = 5.0f,
            .upperBound = 300.0f,
            .epsilon = 1.0f,
        },
        .encoderRatio = 5 * 14 / M_TWOPI,  // 5mm per tooth, 14 teeth
        .posPidConfig = GANTRY_EXTENSION_PID_CONFIG,
        .maxOutput = GANTRY_EXTENSION_PID_CONFIG.maxOutput,
        .staticFeedforward = 0.0f,
    },
    .home = 0.0f,
    .homingSpeed = 10.0f,
    .homingReversed = false,
};

}  // namespace aruwsrc::engineer
#endif  // ENGINEER_GANTRY_CONSTANTS_HPP_   `