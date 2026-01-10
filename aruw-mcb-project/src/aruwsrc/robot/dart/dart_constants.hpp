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

#include "aruwsrc/control/joint/homing/trigger_homed_dual_joint_subsystem.hpp"
namespace aruwsrc::dart
{
static constexpr tap::motor::MotorId UPPER_PULL_MOTOR_ID = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId LOWER_PULL_MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::motor::MotorId DEAD_MOTOR1 = tap::motor::MOTOR5;
static constexpr tap::motor::MotorId DEAD_MOTOR2 = tap::motor::MOTOR4;
static constexpr tap::can::CanBus LAUNCHER_CAN_BUS = tap::can::CanBus::CAN_BUS2;
static constexpr int32_t MANUAL_RELEASE_DESIRED_OUTPUT = -5000;
static constexpr int32_t MANUAL_PULLBACK_DESIRED_OUTPUT = 5000;
static constexpr float MANUAL_PULLBACK_SPEED_MULTIPLIER = 3.0f;
static constexpr int32_t PULLBACK_PULL_POSITION = 0;  // TODO: FIND
static constexpr int32_t RELEASE_POSITION = 0;        // TODO: FIND
//  * @param[in] pwmRampSpeed The speed in PWM percent per millisecond.

static constexpr float SERVO_MIN = 0.5f;
static constexpr float SERVO_MAX = 0.99f;
static constexpr float SERVO_SPEED = 1.0f;
static constexpr tap::gpio::Pwm::Pin SERVO_PORT = tap::gpio::Pwm::Pin::X;
static constexpr tap::gpio::Digital::InputPin BEAMBREAK_PORT = tap::gpio::Digital::InputPin::B;
static constexpr tap::gpio::Digital::InputPin LIMITSWITCH_PORT =
    tap::gpio::Digital::InputPin::D;  // TODO: update value when limit switch is installed on dart

static constexpr aruwsrc::control::joint::homing::TriggerHomedJointSubsystem::Config
    PULL_MOTOR_CONFIG{// TODO: TUNE VALUES
                      .super =
                          {
                              .lowerBound = 0.0f,
                              .upperBound = 500.0f,
                              .epsilon = 1.0,
                              .posPidConfig{
                                  .kp = 10.0f,  // TODO: TUNE THIS
                                  .ki = 0.0f,
                                  .kd = 3.0f,
                                  .maxICumulative = 0.0f,
                                  .maxOutput = 5000.0f},  // these max outs seem safe for now
                              .maxOutput = 3000.0f,

                          },
                      .home = 0.0f,
                      .homingSpeed = 50.0f,
                      .homingReversed = true};

}  // namespace aruwsrc::dart
#endif
