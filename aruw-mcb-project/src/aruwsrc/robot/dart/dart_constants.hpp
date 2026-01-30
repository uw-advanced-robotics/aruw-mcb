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
#include "aruwsrc/control/joint/homing/trigger_homed_joint_subsystem.hpp"
#include "aruwsrc/control/joint/joint_subsystem.hpp"
namespace aruwsrc::dart
{
static constexpr tap::motor::MotorId UPPER_PULL_MOTOR_ID = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId LOWER_PULL_MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::motor::MotorId DEAD_MOTOR1 = tap::motor::MOTOR5;
static constexpr tap::motor::MotorId DEAD_MOTOR2 = tap::motor::MOTOR4;
static constexpr tap::can::CanBus LAUNCHER_CAN_BUS = tap::can::CanBus::CAN_BUS2;
static constexpr int32_t MANUAL_RELEASE_DESIRED_OUTPUT = -5000;
static constexpr int32_t MANUAL_PULLBACK_DESIRED_OUTPUT = 5000;

static constexpr float YAW_INPUT_SENSITIVITY = 0.05;
static constexpr tap::gpio::Digital::InputPin YAW_LIMITSWITCH_PORT =
    tap::gpio::Digital::InputPin::D;
static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR5;
static constexpr float YAW_MOTOR_GEAR_RATIO = tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508;
static constexpr float YAW_LEADSCREW_THREAD_PITCH = 0.002;       // 2 mm
static constexpr float DART_LAUNCHER_YAW_RADIAL_LENGTH = 0.763;  // 76.3 cm

static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 0.0f,  // should be in realm of 10000s
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C620,
    .errDeadzone = 0.002f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr aruwsrc::control::joint::homing::TriggerHomedJointSubsystem::Config
    YAW_HOME_CONFIG = {
        .super =
            {
                .lowerBound = 0.0f,
                .upperBound = 0.2785337f,
                .epsilon = 0.002f,
                .maxSetpointIncrement = 0.05f,  // TODO: adjust
                .initSetpoint = 0.0f,

                .encoderRatio = YAW_MOTOR_GEAR_RATIO * YAW_LEADSCREW_THREAD_PITCH,
                .posPidConfig = YAW_PID_CONFIG,
                .maxOutput = YAW_PID_CONFIG.maxOutput,
                .staticFeedforward = 0,
            },
        .home = 0.0f,
        .homingSpeed = 0.03,     // MAYBE CHANGE
        .homingReversed = false  // TODO: CHANGE IF HOMES THE WRONG WAY
};
static constexpr float MANUAL_PULLBACK_SPEED_MULTIPLIER = 3.0f;

static constexpr int32_t PULLBACK_PULL_POSITION = 0;  // TODO: FIND
static constexpr int32_t RELEASE_POSITION = 0;        // TODO: FIND
static constexpr int32_t GRAB_POSITION = 0;           // TODO: FIND
static constexpr int32_t RELOAD_POSITION = 0;         // TODO: FIND
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
                                  .kp = 80.0f,  // TODO: TUNE THIS
                                  .ki = 0.0f,
                                  .kd = 20.0f,
                                  .maxICumulative = 0.0f,
                                  .maxOutput = 5000.0f},  // these max outs seem safe for now
                              .maxOutput = 3000.0f,

                          },
                      .home = 0.0f,
                      .homingSpeed = 50.0f,
                      .homingReversed = true};

}  // namespace aruwsrc::dart
#endif
