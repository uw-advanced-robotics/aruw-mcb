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

#include "tap/communication/gpio/digital.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/servo.hpp"
#include "aruwsrc/control/joint/joint_subsystem.hpp"

namespace aruwsrc::robot::dart
{
static constexpr tap::motor::MotorId UPPER_PULL_MOTOR_ID = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId LOWER_PULL_MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::motor::MotorId DEAD_MOTOR1 = tap::motor::MOTOR5;
static constexpr tap::motor::MotorId DEAD_MOTOR2 = tap::motor::MOTOR4;
static constexpr tap::can::CanBus LAUNCHER_CAN_BUS = tap::can::CanBus::CAN_BUS2;
static constexpr int32_t MANUAL_RELEASE_DESIRED_OUTPUT = -5000;
static constexpr int32_t MANUAL_PULLBACK_DESIRED_OUTPUT = 5000;

static constexpr tap::gpio::Digital::InputPin YAW_LIMITSWITCH_PORT = tap::gpio::Digital::InputPin::B; //TODO: correct constant
static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR3; // PLACEHOLDER
static constexpr float YAW_MOTOR_GEAR_RATIO = tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508;
static constexpr float YAW_LEADSCREW_THREAD_PITCH = 0.002; // 2 mm
static constexpr float DART_LAUNCHER_YAW_RADIAL_LENGTH = 0;

static constexpr aruwsrc::control::joint::homing::TriggerHomedJointSubsystem::Config YAW_HOME_CONFIG = {
    .super = {
        .lowerBound = 0.0f, 
        .upperBound = 0.0f,
        .epsilon = 1.0f,
        .maxSetpointIncrement = upperbound - lowerbound, // is this chill?
        .initSetpoint = 0.0f,

        .encoderRatio = YAW_MOTOR_GEAR_RATIO * YAW_LEADSCREW_THREAD_PITCH,
        .posPidConfig = YAW_PID_CONFIG,
        .maxOutput = YAW_PID_CONFIG.maxOutput,
        .staticFeedforward = 0,
    },
    .home = 0.0f,
    .homingSpeed = 10.0f, // MAYBE CHANGE
    .homingReversed = false // TODO: CHANGE IF HOMES THE WRONG WAY
};

static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 16'000.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

//  * @param[in] pwmPin The pin to attach the Servo class with.
//  * @param[in] maximumPwm The maximum allowable PWM output. This is limited between 0 and 1.
//  * @param[in] minimumPwm The minimum allowable PWM output. This is limited between 0 and 1.
//  * @param[in] pwmRampSpeed The speed in PWM percent per millisecond.

static constexpr float SERVO_MIN = 0.5f;
static constexpr float SERVO_MAX = 0.99f;
static constexpr float SERVO_SPEED = 1.0f;
static constexpr tap::gpio::Pwm::Pin SERVO_PORT = tap::gpio::Pwm::Pin::X;
static constexpr tap::gpio::Digital::InputPin BEAMBREAK_PORT = tap::gpio::Digital::InputPin::B;
static constexpr tap::gpio::Digital::InputPin LIMITSWITCH_PORT =
    tap::gpio::Digital::InputPin::D;  // TODO: update value when limit switch is installed on dart

}  // namespace aruwsrc::robot::dart
#endif
