/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef ENGINEER_CUBE_STORAGE_CONSTANTS_HPP_
#define ENGINEER_CUBE_STORAGE_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/gpio/digital.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/joint/homing/trigger_homed_joint_subsystem.hpp"

namespace aruwsrc::engineer::cube_storage
{
static constexpr tap::motor::MotorId CUBE_STORAGE_MOTOR_ID = tap::motor::MOTOR7;

static constexpr tap::can::CanBus CUBE_STORAGE_MOTOR_CAN_BUS = tap::can::CanBus::CAN_BUS2;

static constexpr tap::gpio::Digital::InputPin CUBE_STORAGE_LIMITSWITCH_PORT =
    tap::gpio::Digital::InputPin::B;

static constexpr float MM_PER_REVOLUTION = 74.63f / M_TWOPI; // update

static constexpr tap::algorithms::SmoothPidConfig CUBE_STORAGE_MOTOR_PID_CONFIG = {
    .kp = 300.0f,
    .ki = 50.0f,
    .kd = 25.0f,
    .maxICumulative = 2000.0f,
    .maxOutput = 6000.0f,
};

static constexpr float CUBE_STORAGE_MOVE_SPEED = -2.0f;

static constexpr float CUBE_STORAGE_LEFT_SETPOINT = -40.0f;
static constexpr float CUBE_STORAGE_CENTER_SETPOINT = -220.0f;
static constexpr float CUBE_STORAGE_RIGHT_SETPOINT = -310.0f;

static constexpr aruwsrc::control::joint::homing::TriggerHomedJointSubsystem::Config
    CUBE_STORAGE_CONFIG{
        .super =  // JointSubsystem::Config
        {
            .lowerBound = -320.0f,
            .upperBound = -40.0f,
            .epsilon = 0.5f,
            .encoderRatio = MM_PER_REVOLUTION,
            .posPidConfig = CUBE_STORAGE_MOTOR_PID_CONFIG,
            .maxOutput = CUBE_STORAGE_MOTOR_PID_CONFIG.maxOutput,
            .staticFeedforward = 0.0f,
        },
        .home = CUBE_STORAGE_CENTER_SETPOINT,
        .homingSpeed = 20.0f,
        .homingReversed = true,
    };

}  // namespace aruwsrc::engineer::cube_storage
#endif