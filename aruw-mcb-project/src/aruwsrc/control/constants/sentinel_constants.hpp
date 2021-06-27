/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SENTINEL_CONSTANTS_HPP_
#define SENTINEL_CONSTANTS_HPP_

#ifndef ROBOT_CONSTANTS_HPP_
#error "Don't include this file directly, include robot_constants.hpp instead"
#endif

#include "aruwlib/communication/can/can_bus.hpp"
#include "aruwlib/communication/gpio/digital.hpp"
#include "aruwlib/communication/gpio/pwm.hpp"
#include "aruwlib/motor/dji_motor.hpp"

// For comments, see constants.md
namespace sentinel_control::constants
{
// motor and CAN bus constants
namespace motor
{
// CAN 1
static constexpr aruwlib::motor::MotorId LAUNCHER_LEFT_MOTOR_ID_LOWER = aruwlib::motor::MOTOR1;
static constexpr aruwlib::motor::MotorId LAUNCHER_RIGHT_MOTOR_ID_LOWER = aruwlib::motor::MOTOR2;
static constexpr aruwlib::motor::MotorId LAUNCHER_RIGHT_MOTOR_ID_UPPER = aruwlib::motor::MOTOR3;
static constexpr aruwlib::motor::MotorId LAUNCHER_LEFT_MOTOR_ID_UPPER = aruwlib::motor::MOTOR4;
static constexpr aruwlib::motor::MotorId CHASSIS_RIGHT_MOTOR_ID = aruwlib::motor::MOTOR5;
static constexpr aruwlib::motor::MotorId CHASSIS_LEFT_MOTOR_ID = aruwlib::motor::MOTOR6;
static constexpr aruwlib::motor::MotorId AGITATOR_MOTOR_ID = aruwlib::motor::MOTOR7;
}  // namespace motor

namespace can
{
static constexpr aruwlib::can::CanBus LAUNCHER_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;
static constexpr aruwlib::can::CanBus CHASSIS_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;
static constexpr aruwlib::can::CanBus AGITATOR_MOTOR_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;
}  // namespace can

namespace gpio
{
static constexpr aruwlib::gpio::Pwm::Pin SWITCHER_SERVO_PIN = aruwlib::gpio::Pwm::Pin::W;
static constexpr aruwlib::gpio::Analog::Pin CURRENT_SENSOR_PIN = aruwlib::gpio::Analog::Pin::S;
static constexpr aruwlib::gpio::Digital::InputPin LEFT_LIMIT_SWITCH =
    aruwlib::gpio::Digital::InputPin::A;
static constexpr aruwlib::gpio::Digital::InputPin RIGHT_LIMIT_SWITCH =
    aruwlib::gpio::Digital::InputPin::B;
}  // namespace gpio

namespace chassis
{
static constexpr aruwlib::gpio::Digital::InputPin LEFT_LIMIT_SWITCH =
    aruwlib::gpio::Digital::InputPin::A;
static constexpr aruwlib::gpio::Digital::InputPin RIGHT_LIMIT_SWITCH =
    aruwlib::gpio::Digital::InputPin::B;

static constexpr float CHASSIS_PID_P = 5.0f;
static constexpr float CHASSIS_PID_I = 0.0f;
static constexpr float CHASSIS_PID_D = 0.1f;
static constexpr float CHASSIS_PID_MAX_ERROR_SUM = 0.0f;
static constexpr float CHASSIS_PID_MAX_OUTPUT = 16000;

// radius of the wheel in mm
static constexpr float WHEEL_RADIUS = 35.0f;
static constexpr float GEAR_RATIO = GEAR_RATIO_GM3508;
}  // namespace chassis

namespace agitator
{
// position PID terms
// PID terms for sentinel
static constexpr float AGITATOR_PID_17MM_P = 170000.0f;
static constexpr float AGITATOR_PID_17MM_I = 0.0f;
static constexpr float AGITATOR_PID_17MM_D = 80.0f;
static constexpr float AGITATOR_PID_17MM_MAX_ERR_SUM = 0.0f;
static constexpr float AGITATOR_PID_17MM_MAX_OUT = 16000.0f;
static constexpr float AGITATOR_MOTOR_INVERTED = false;
static constexpr float AGITATOR_GEAR_RATIO = 36.0f;
static constexpr float JAMMING_DISTANCE = 0;
static constexpr float JAMMING_TIME = 0;
}  // namespace agitator

namespace turret
{
static constexpr float TURRET_START_ANGLE = 90.0f;
static constexpr float TURRET_YAW_MIN_ANGLE = TURRET_START_ANGLE - 90.0f;
static constexpr float TURRET_YAW_MAX_ANGLE = TURRET_START_ANGLE + 90.0f;
static constexpr float TURRET_PITCH_MIN_ANGLE = TURRET_START_ANGLE - 13.0f;
static constexpr float TURRET_PITCH_MAX_ANGLE = TURRET_START_ANGLE + 20.0f;
static constexpr aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;
static constexpr aruwlib::motor::MotorId PITCH_MOTOR_ID = aruwlib::motor::MOTOR6;
static constexpr aruwlib::motor::MotorId YAW_MOTOR_ID = aruwlib::motor::MOTOR5;
static constexpr uint16_t YAW_START_ENCODER_POSITION = 8160;
static constexpr uint16_t PITCH_START_ENCODER_POSITION = 4100;
}  // namespace turret

namespace launcher
{
static constexpr float LAUNCHER_PID_P = 30.0f;
static constexpr float LAUNCHER_PID_I = 0.0f;
static constexpr float LAUNCHER_PID_D = 5.0f;
static constexpr float LAUNCHER_PID_MAX_ERROR_SUM = 0.0f;
static constexpr float LAUNCHER_PID_MAX_OUTPUT = 16000.0f;

static constexpr float SWITCHER_UPPER_PWM = 0.13f;
static constexpr float SWITCHER_LOWER_PWM = 0.19f;
}  // namespace launcher
}  // namespace sentinel_control::constants

#endif  // SENTINEL_CONSTANTS_HPP_
