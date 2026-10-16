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

#ifndef ENGINEER_EXTENSION_CONSTANTS_HPP_
#define ENGINEER_EXTENSION_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/gpio/digital.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/joint/homing/trigger_homed_joint_subsystem.hpp"

namespace aruwsrc::engineer
{
static constexpr tap::can::CanBus CAN_BUS_EXTENSION = tap::can::CanBus::CAN_BUS1;

static constexpr tap::motor::MotorId EXTENSION_MOTOR_ID = tap::motor::MotorId::MOTOR3;

static constexpr tap::algorithms::SmoothPidConfig EXTENSION_PID_CONFIG = {
    .kp = 0.0f,  // 300
    .ki = 0.0f,
    .kd = 0.0f,  // 40
    .maxICumulative = 0.0f,
    .maxOutput = 0.0f,  // 2000
};

static constexpr aruwsrc::control::joint::homing::TriggerHomedJointSubsystem::Config
    EXTENSION_CONFIG{
        .super =  // JointSubsystem::Config
        {
            .lowerBound = 5.0f,
            .upperBound = 300.0f,
            .epsilon = 1.0f,
            .encoderRatio =
                5 * 14 /
                M_TWOPI,  // 5mm per tooth, 14 teeth was old constants, probably change this
            .posPidConfig = EXTENSION_PID_CONFIG,
            .maxOutput = EXTENSION_PID_CONFIG.maxOutput,
            .staticFeedforward = 0.0f,
        },
        .home = 0.0f,
        .homingSpeed = 10.0f,
        .homingReversed = false,
    };

static constexpr float EXTENSION_MOVE_SPEED = 0.6f;

static constexpr tap::gpio::Digital::InputPin EXTENSION_LIMIT_SWITCH_PIN =
    tap::gpio::Digital::InputPin::T;

}  // namespace aruwsrc::engineer
#endif  // ENGINEER_EXTENSION_CONSTANTS_HPP_