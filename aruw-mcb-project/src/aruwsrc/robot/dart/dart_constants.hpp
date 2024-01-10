/*
 * Copyright (c) 2022-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef DART_CONSTANTS_HPP_
#define DART_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/gpio/digital.hpp"
#include "tap/motor/dji_motor.hpp"

namespace aruwsrc::control::turret
{
/** Motor IDs */
static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS2;

static constexpr tap::motor::MotorId LAUNCHER_PULL_MOTOR_1_ID = tap::motor::MOTOR3;
static constexpr tap::motor::MotorId LAUNCHER_PULL_MOTOR_2_ID = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId LAUNCHER_ENCODER_MOTOR_ID = tap::motor::MOTOR5;

static constexpr tap::gpio::Digital::OutputPin LAUNCHER_ACTUATOR_RELEASE_PIN = tap::gpio::Digital::OutputPin::G;
static constexpr tap::gpio::Digital::OutputPin LAUNCHER_ACTUATOR_LOCK_PIN = tap::gpio::Digital::OutputPin::H;

static constexpr tap::motor::MotorId PIVOT_MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::motor::MotorId PIVOT_DEAD_MOTOR_ID = tap::motor::MOTOR4;

static constexpr tap::motor::MotorId LOADER_TOP_MOTOR_ID = tap::motor::MOTOR6;
static constexpr tap::motor::MotorId LOADER_MIDDLE_MOTOR_ID = tap::motor::MOTOR8;
static constexpr tap::motor::MotorId LOADER_BOTTOM_MOTOR_ID = tap::motor::MOTOR7;

/** Kinematic Constants */

static constexpr int64_t LAUNCHER_ENCODER_MOTOR_AT_TOP = 0; ///< Encoder val at which the cart is at the top of the launcher
static constexpr uint64_t LAUNCHER_ENCODER_MOTOR_AT_TOP_TOLERANCE = 5;
static constexpr int16_t LAUNCHER_PULL_MOTORS_SLOW_RELEASE_SPEED = 8000;

static constexpr uint16_t HOMING_SPEED = 250.0f;

/** PID Configs */

static constexpr tap::algorithms::SmoothPidConfig pivotPID = {
    .kp = 10.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 5000.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig launcherPullPID = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig loaderPID = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

}  // namespace aruwsrc::control::turret
#endif
