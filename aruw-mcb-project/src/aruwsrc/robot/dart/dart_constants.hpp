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
#include "tap/algorithms/smooth_pid.hpp"
namespace aruwsrc::robot::dart
{
static constexpr tap::motor::MotorId UPPER_PULL_MOTOR_ID = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId LOWER_PULL_MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::motor::MotorId RELOADER_MOTOR_ID = tap::motor::MOTOR6; //TODO: actually put the value 
static constexpr tap::motor::MotorId DEAD_MOTOR1 = tap::motor::MOTOR5;
static constexpr tap::motor::MotorId DEAD_MOTOR2 = tap::motor::MOTOR4;
static constexpr tap::can::CanBus LAUNCHER_CAN_BUS = tap::can::CanBus::CAN_BUS2;
static constexpr int32_t MANUAL_RELEASE_DESIRED_OUTPUT = -5000;
static constexpr int32_t MANUAL_PULLBACK_DESIRED_OUTPUT = 5000;
static constexpr tap::algorithms::SmoothPidConfig DART_RELOADER_PID_CONFIG = {
    //TODO tune PID values
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .errDeadzone = 0.0f,

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
