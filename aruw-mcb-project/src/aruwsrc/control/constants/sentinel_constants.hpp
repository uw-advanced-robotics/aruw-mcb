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
#include "aruwlib/communication/gpio/analog.hpp"
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
static constexpr aruwlib::motor::MotorId PITCH_MOTOR_ID_RIGHT = aruwlib::motor::MOTOR5;
static constexpr aruwlib::motor::MotorId YAW_MOTOR_ID = aruwlib::motor::MOTOR6;
static constexpr aruwlib::motor::MotorId AGITATOR_MOTOR_ID = aruwlib::motor::MOTOR7;
static constexpr aruwlib::motor::MotorId PITCH_MOTOR_ID_LEFT = aruwlib::motor::MOTOR8;

// CAN 2
static constexpr aruwlib::motor::MotorId CHASSIS_RIGHT_MOTOR_ID = aruwlib::motor::MOTOR5;
static constexpr aruwlib::motor::MotorId CHASSIS_LEFT_MOTOR_ID = aruwlib::motor::MOTOR6;
}  // namespace motor

namespace can
{
static constexpr aruwlib::can::CanBus LAUNCHER_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;
static constexpr aruwlib::can::CanBus CHASSIS_CAN_BUS = aruwlib::can::CanBus::CAN_BUS2;
static constexpr aruwlib::can::CanBus AGITATOR_MOTOR_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;
static constexpr aruwlib::can::CanBus TURRET_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;
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
static constexpr float GEAR_RATIO = 19.0f;

/// @see power_limiter.hpp for what these mean
static constexpr float MAX_ENERGY_BUFFER = 200.0f;
static constexpr float ENERGY_BUFFER_LIMIT_THRESHOLD = 100.0f;
static constexpr float ENERGY_BUFFER_CRIT_THRESHOLD = 0;
static constexpr uint16_t POWER_CONSUMPTION_THRESHOLD = 5;
static constexpr float CURRENT_ALLOCATED_FOR_ENERGY_BUFFER_LIMITING = 15000;

// RMUL length of the rail, in mm
static constexpr float RAIL_LENGTH = 2130;
// Our length of the rail, in mm
// static constexpr float RAIL_LENGTH = 1900;
static constexpr float SENTINEL_LENGTH = 480;
static constexpr float RAMP_SPEED = 10.0f;
static constexpr float MAX_DESIRED_TRAVERSE_SPEED = 3000.0f;
static constexpr float TURNAROUND_BUFFER = 0.2f * RAIL_LENGTH;
static constexpr int16_t RANDOM_DRIVE_MIN_RPM = 5000;
static constexpr int16_t RANDOM_DRIVE_MAX_RPM = 7000;
static constexpr int16_t RANDOM_DRIVE_CHANGE_TIME_INTERVAL = 750;
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
static constexpr float JAMMING_DISTANCE = aruwlib::algorithms::PI / 10.0f;
static constexpr float JAMMING_TIME = 150;
}  // namespace agitator

namespace turret
{
static constexpr float TURRET_YAW_START_ANGLE = 90.0f;
static constexpr float TURRET_YAW_MIN_ANGLE = 5.0f;
static constexpr float TURRET_YAW_MAX_ANGLE = 175.0f;
static constexpr float TURRET_PITCH_START_ANGLE = 62.0f;
static constexpr float TURRET_PITCH_MIN_ANGLE = 45.0f;
static constexpr float TURRET_PITCH_MAX_ANGLE = 90.0f;

static constexpr float YAW_P = 2000.0f;
static constexpr float YAW_I = 0.0f;
static constexpr float YAW_D = 120.0f;
static constexpr float YAW_MAX_ERROR_SUM = 0.0f;
static constexpr float YAW_MAX_OUTPUT = 30000.0f;
static constexpr float YAW_Q_DERIVATIVE_KALMAN = 1.0f;
static constexpr float YAW_R_DERIVATIVE_KALMAN = 30.0f;
static constexpr float YAW_Q_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float YAW_R_PROPORTIONAL_KALMAN = 0.0f;

static constexpr float PITCH_P = 1300.0f;
static constexpr float PITCH_I = 0.0f;
static constexpr float PITCH_D = 80.0f;
static constexpr float PITCH_MAX_ERROR_SUM = 0.0f;
static constexpr float PITCH_MAX_OUTPUT = 30000.0f;
static constexpr float PITCH_Q_DERIVATIVE_KALMAN = 1.5f;
static constexpr float PITCH_R_DERIVATIVE_KALMAN = 30.0f;
static constexpr float PITCH_Q_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float PITCH_R_PROPORTIONAL_KALMAN = 2.0f;

static constexpr float USER_YAW_INPUT_SCALAR = 0.75f;
static constexpr float USER_PITCH_INPUT_SCALAR = 0.5f;

static constexpr float PITCH_GRAVITY_COMPENSATION_KP = 0.0f;

static constexpr uint16_t YAW_START_ENCODER_POSITION = 0;
static constexpr uint16_t PITCH_90DEG_ENCODER_POSITION_LEFT = 0;
static constexpr uint16_t PITCH_90DEG_ENCODER_POSITION_RIGHT = 0;
}  // namespace turret

namespace launcher
{
static constexpr float LAUNCHER_PID_P = 30.0f;
static constexpr float LAUNCHER_PID_I = 0.0f;
static constexpr float LAUNCHER_PID_D = 5.0f;
static constexpr float LAUNCHER_PID_MAX_ERROR_SUM = 0.0f;
static constexpr float LAUNCHER_PID_MAX_OUTPUT = 16000.0f;
static constexpr float FRICTION_WHEEL_RAMP_SPEED = 0.5f;

static constexpr float FRICTION_WHEEL_TARGET_RPM = 5150;

static constexpr float SWITCHER_UPPER_PWM = 0.13f;
static constexpr float SWITCHER_LOWER_PWM = 0.19f;
}  // namespace launcher
}  // namespace sentinel_control::constants

#endif  // SENTINEL_CONSTANTS_HPP_
