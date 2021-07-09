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

#ifndef SOLDIER_CONSTANTS_HPP_
#define SOLDIER_CONSTANTS_HPP_

#ifndef ROBOT_CONSTANTS_HPP_
#error "Don't include this file directly, include robot_constants.hpp instead"
#endif

#include "aruwlib/algorithms/smooth_pid.hpp"
#include "aruwlib/communication/can/can_bus.hpp"
#include "aruwlib/communication/gpio/analog.hpp"
#include "aruwlib/control/chassis/power_limiter.hpp"
#include "aruwlib/motor/dji_motor.hpp"

#include "aruwsrc/control/chassis/chassis_subsystem.hpp"

// For comments, see constants.md
namespace soldier_control::constants
{
// motor and CAN bus constants
namespace motor
{
// CAN 1
static constexpr aruwlib::motor::MotorId LAUNCHER_LEFT_MOTOR_ID = aruwlib::motor::MOTOR1;
static constexpr aruwlib::motor::MotorId LAUNCHER_RIGHT_MOTOR_ID = aruwlib::motor::MOTOR2;
static constexpr aruwlib::motor::MotorId YAW_MOTOR_ID = aruwlib::motor::MOTOR5;
static constexpr aruwlib::motor::MotorId PITCH_MOTOR_ID = aruwlib::motor::MOTOR6;
static constexpr aruwlib::motor::MotorId HOPPER_COVER_MOTOR_ID = aruwlib::motor::MOTOR8;
static constexpr aruwlib::motor::MotorId AGITATOR_MOTOR_ID = aruwlib::motor::MOTOR7;

// CAN 2
static constexpr aruwlib::motor::MotorId RIGHT_FRONT_MOTOR_ID = aruwlib::motor::MOTOR1;
static constexpr aruwlib::motor::MotorId LEFT_FRONT_MOTOR_ID = aruwlib::motor::MOTOR2;
static constexpr aruwlib::motor::MotorId LEFT_BACK_MOTOR_ID = aruwlib::motor::MOTOR3;
static constexpr aruwlib::motor::MotorId RIGHT_BACK_MOTOR_ID = aruwlib::motor::MOTOR4;
}  // namespace motor

namespace can
{
static constexpr aruwlib::can::CanBus LAUNCHER_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;
static constexpr aruwlib::can::CanBus TURRET_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;
static constexpr aruwlib::can::CanBus AGITATOR_MOTOR_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;
static constexpr aruwlib::can::CanBus HOPPER_COVER_MOTOR_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;

static constexpr aruwlib::can::CanBus CHASSIS_CAN_BUS = aruwlib::can::CanBus::CAN_BUS2;
}  // namespace can

namespace gpio
{
static constexpr aruwlib::gpio::Analog::Pin CURRENT_SENSOR_PIN = aruwlib::gpio::Analog::Pin::S;
}

// PID and mechanical constants

namespace agitator
{
// position PID terms
// PID terms for soldier
static constexpr aruwlib::algorithms::PidConfigStruct AGITATOR_PID_CONFIG =
    {170000.0f, 0.0f, 80.0f, 0.0f, 16000.0f, 1.0f, 0.0f, 1.0f, 0.0f};
static constexpr float AGITATOR_GEARBOX_RATIO = 36.0f;
static constexpr bool AGITATOR_INVERTED = false;
static constexpr float AGITATOR_JAMMING_DISTANCE = 1.0f;
static constexpr uint32_t AGITATOR_JAMMING_TIME = 250;
static constexpr bool IS_HOPPER_COVER_INVERTED = false;
static constexpr float HOPPER_COVER_JAMMING_DISTANCE = 1.0f;
static constexpr uint32_t HOPPER_COVER_JAMMING_TIME = 250;
}  // namespace agitator

namespace chassis
{
static constexpr int MAX_WHEEL_SPEED_SINGLE_MOTOR = 7000;

static constexpr aruwlib::algorithms::PidConfigStruct CHASSIS_PID_CONFIG =
    {20.0f, 0.0f, 0.0f, 0.0f, 16000.0f, 0.0f, 0.0f, 0.0f, 0.0f};

static constexpr float CHASSIS_REVOLVE_PID_MAX_P = MAX_WHEEL_SPEED_SINGLE_MOTOR;
static constexpr float CHASSIS_REVOLVE_PID_KD = 500.0f;
static constexpr float CHASSIS_REVOLVE_PID_MAX_D = 0.0f;
static constexpr int CHASSIS_REVOLVE_PID_MIN_ERROR_ROTATION_D = 0;
static constexpr float CHASSIS_REVOLVE_PID_MAX_OUTPUT = 5000.0f;
static constexpr float MIN_ROTATION_THRESHOLD = 800.0f;

static constexpr aruwsrc::chassis::ChassisSubsystem::ChassisMechanicalConstants
    CHASSIS_MECHANICAL_CONSTANTS = {1.0f / 19.0f, 0.366f, 0.366f, 0.076f, 0.0f, 0.0f};

static constexpr float CHASSIS_AUTOROTATE_PID_KP = -125.0f;

static constexpr aruwlib::control::chassis::PowerLimiterConfig CHASSIS_POWER_LIMIT_CONFIG =
    {60.0f, 40.0f, 5, 20, 30000};

static constexpr float WIGGLE_PERIOD = 1600.0f;
static constexpr float WIGGLE_MAX_ROTATE_ANGLE = 60.0f;
static constexpr float WIGGLE_ROTATE_KP = -250.0f;
static constexpr float WIGGLE_TRANSLATIONAL_SPEED_FRACTION = 0.5f;
static constexpr float WIGGLE_OUT_OF_CENTER_MAX_ROTATE_ERR = 10.0f;

static constexpr float BEYBLADE_TRANSLATIONAL_SPEED_FRACTION = 0.5f;
static constexpr float BEYBLADE_TRANSLATION_MIN_UNTIL_SLOW_ROTATION = 0.5f;
static constexpr float BEYBLADE_ROTATION_WHEELSPEED_NOT_TRANSLATING = 7000;
static constexpr float BEYBLADE_ROTATION_WHEELSPEED_TRANSLATING = 3500;
}  // namespace chassis

namespace turret
{
// Subsystem
static constexpr float TURRET_START_ANGLE = 90.0f;
static constexpr float TURRET_YAW_MIN_ANGLE = TURRET_START_ANGLE - 90.0f;
static constexpr float TURRET_YAW_MAX_ANGLE = TURRET_START_ANGLE + 90.0f;
static constexpr float TURRET_PITCH_MIN_ANGLE = TURRET_START_ANGLE - 13.0f;
static constexpr float TURRET_PITCH_MAX_ANGLE = TURRET_START_ANGLE + 20.0f;

static constexpr uint16_t YAW_START_ENCODER_POSITION = 8160;
static constexpr uint16_t PITCH_START_ENCODER_POSITION = 4100;
static constexpr float FEED_FORWARD_KP = 0.0f;  // TODO tune this value
static constexpr float FEED_FORWARD_MAX_OUTPUT = 20000.0f;

// CV control
static constexpr aruwlib::algorithms::PidConfigStruct CV_YAW_PID_CONFIG =
    {4500.0f, 0.0f, 140.0f, 0.0f, 32000.0f, 1.0f, 20.0f, 1.0f, 0.0f};
static constexpr aruwlib::algorithms::PidConfigStruct CV_PITCH_PID_CONFIG =
    {3500.0f, 0.0f, 80.0f, 0.0f, 32000.0f, 1.5f, 20.0f, 1.0f, 2.0f};

// World relative control
static constexpr float YAW_WR_TURRET_IMU_D = 180.0f;
static constexpr float YAW_WR_CHASSIS_IMU_D = 180.0f;
static constexpr aruwlib::algorithms::PidConfigStruct WR_YAW_PID_CONFIG =
    {4000.0f, 0.0f, YAW_WR_TURRET_IMU_D, 0.0f, 30000.0f, 1.0f, 20.0f, 1.0f, 10.0f};
static constexpr aruwlib::algorithms::PidConfigStruct WR_PITCH_PID_CONFIG =
    {4500.0f, 0.0f, 90.0f, 0.0f, 30000.0f, 1.5f, 20.0f, 1.0f, 2.0f};

static constexpr float USER_YAW_INPUT_SCALAR = 1.0f;
static constexpr float USER_PITCH_INPUT_SCALAR = 0.5f;

static constexpr float PITCH_GRAVITY_COMPENSATION_KP = 4000.0f;
}  // namespace turret

namespace launcher
{
static constexpr aruwlib::algorithms::PidConfigStruct LAUNCHER_PID_CONFIG =
    {30.0f, 0.0f, 5.0f, 0.0f, 16000.0f, 1.0f, 0.0f, 1.0f, 0.0f};
static constexpr float FRICTION_WHEEL_RAMP_SPEED = 0.5f;
}  // namespace launcher
}  // namespace soldier_control::constants

#endif  // SOLDIER_CONSTANTS_HPP_
