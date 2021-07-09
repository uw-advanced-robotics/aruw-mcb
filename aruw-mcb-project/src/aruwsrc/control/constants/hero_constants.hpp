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

#ifndef HERO_CONSTANTS_HPP_
#define HERO_CONSTANTS_HPP_

#ifndef ROBOT_CONSTANTS_HPP_
#error "Don't include this file directly, include robot_constants.hpp instead"
#endif

#include "aruwlib/algorithms/smooth_pid.hpp"
#include "aruwlib/communication/can/can_bus.hpp"
#include "aruwlib/communication/gpio/digital.hpp"
#include "aruwlib/control/chassis/power_limiter.hpp"
#include "aruwlib/motor/dji_motor.hpp"

#include "aruwsrc/control/chassis/chassis_subsystem.hpp"

// For comments, see constants.md
namespace hero_control::constants
{
// motor and CAN bus constants
namespace motor
{
// CAN 1
static constexpr aruwlib::motor::MotorId LAUNCHER_RIGHT_MOTOR_ID = aruwlib::motor::MOTOR1;
static constexpr aruwlib::motor::MotorId LAUNCHER_LEFT_MOTOR_ID = aruwlib::motor::MOTOR2;
static constexpr aruwlib::motor::MotorId WATERWHEEL_MOTOR_ID = aruwlib::motor::MOTOR3;
static constexpr aruwlib::motor::MotorId YAW_MOTOR_ID = aruwlib::motor::MOTOR5;
static constexpr aruwlib::motor::MotorId PITCH_MOTOR_ID = aruwlib::motor::MOTOR6;
static constexpr aruwlib::motor::MotorId KICKER1_MOTOR_ID = aruwlib::motor::MOTOR7;
static constexpr aruwlib::motor::MotorId KICKER2_MOTOR_ID = aruwlib::motor::MOTOR8;

// CAN 2
static constexpr aruwlib::motor::MotorId RIGHT_FRONT_MOTOR_ID = aruwlib::motor::MOTOR1;
static constexpr aruwlib::motor::MotorId LEFT_FRONT_MOTOR_ID = aruwlib::motor::MOTOR2;
static constexpr aruwlib::motor::MotorId LEFT_BACK_MOTOR_ID = aruwlib::motor::MOTOR3;
static constexpr aruwlib::motor::MotorId RIGHT_BACK_MOTOR_ID = aruwlib::motor::MOTOR4;
}  // namespace motor

namespace can
{
static constexpr aruwlib::can::CanBus WATERWHEEL_MOTOR_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;
static constexpr aruwlib::can::CanBus LAUNCHER_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;
static constexpr aruwlib::can::CanBus TURRET_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;
static constexpr aruwlib::can::CanBus KICKER1_MOTOR_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;
static constexpr aruwlib::can::CanBus KICKER2_MOTOR_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;

static constexpr aruwlib::can::CanBus CHASSIS_CAN_BUS = aruwlib::can::CanBus::CAN_BUS2;
}  // namespace can

namespace gpio
{
static constexpr aruwlib::gpio::Analog::Pin CURRENT_SENSOR_PIN = aruwlib::gpio::Analog::Pin::S;
static constexpr aruwlib::gpio::Digital::InputPin WATERWHEEL_LIMIT_SWITCH_PIN =
    aruwlib::gpio::Digital::InputPin::B;
}  // namespace gpio

// PID and mechanical constants

namespace agitator
{
// Hero's waterwheel constants
static constexpr aruwlib::algorithms::PidConfigStruct WATERWHEEL_PID_CONFIG =
    {100000.0f, 0.0f, 10.0f, 0.0f, 16000.0f, 1.0f, 0.0f, 1.0f, 0.0f};
static constexpr float WATERWHEEL_GEARBOX_RATIO = 19.0f;
static constexpr bool WATERWHEEL_INVERTED = false;
/**
 * The jamming constants for waterwheel. Waterwheel is considered jammed if difference between
 * setpoint and current angle is > `JAM_DISTANCE_TOLERANCE_WATERWHEEL` radians for >=
 * `JAM_TEMPORAL_TOLERANCE_WATERWHEEL` ms;
 */
static constexpr float WATERWHEEL_JAM_DISTANCE_TOLERANCE = aruwlib::algorithms::PI / 28.0f;
static constexpr uint32_t WATERWHEEL_JAM_TEMPORAL_TOLERANCE = 100.0f;

// PID terms for the hero kicker
static constexpr aruwlib::algorithms::PidConfigStruct KICKER_PID_CONFIG =
    {50000.0f, 0.0f, 10.0f, 0.0f, 16000.0f, 1.0f, 0.0f, 1.0f, 0.0f};
static constexpr float KICKER_GEARBOX_RATIO = 36.0f;
static constexpr bool KICKER_INVERTED = false;

}  // namespace agitator

namespace chassis
{
static constexpr int MAX_WHEEL_SPEED_SINGLE_MOTOR = 7000;

static constexpr aruwlib::algorithms::PidConfigStruct CHASSIS_PID_CONFIG =
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f};

static constexpr float CHASSIS_REVOLVE_PID_MAX_P = 0.0;
static constexpr float CHASSIS_REVOLVE_PID_KD = 0.0;
static constexpr int CHASSIS_REVOLVE_PID_MIN_ERROR_ROTATION_D = 0;
static constexpr float CHASSIS_REVOLVE_PID_MAX_OUTPUT = 5000.0f;
static constexpr float MIN_ROTATION_THRESHOLD = 800.0f;

static constexpr float CHASSIS_REVOLVE_PID_MAX_D = 0.0f;

static constexpr aruwsrc::chassis::ChassisSubsystem::ChassisMechanicalConstants
    CHASSIS_MECHANICAL_CONSTANTS = {1.0f / 19.0f, 0.517f, 0.600f, 0.076f, 0.175f, 0.0f};

static constexpr float CHASSIS_AUTOROTATE_PID_KP = -125.0f;

static constexpr aruwlib::control::chassis::PowerLimiterConfig CHASSIS_POWER_LIMITER_CONFIG =
    {60.0f, 40.0f, 5, 20, 30000};

static constexpr float WIGGLE_PERIOD = 1600.0f;
static constexpr float WIGGLE_MAX_ROTATE_ANGLE = 60.0f;
static constexpr float WIGGLE_ROTATE_KP = -250.0f;
static constexpr float WIGGLE_TRANSLATIONAL_SPEED_FRACTION = 0.5f;
static constexpr float WIGGLE_OUT_OF_CENTER_MAX_ROTATE_ERR = 10.0f;
}  // namespace chassis

namespace turret
{
static constexpr float TURRET_START_ANGLE = 90.0f;
static constexpr float TURRET_YAW_MIN_ANGLE = TURRET_START_ANGLE - 90.0f;
static constexpr float TURRET_YAW_MAX_ANGLE = TURRET_START_ANGLE + 90.0f;
static constexpr float TURRET_PITCH_MIN_ANGLE = TURRET_START_ANGLE - 13.0f;
static constexpr float TURRET_PITCH_MAX_ANGLE = TURRET_START_ANGLE + 20.0f;

static constexpr uint16_t YAW_START_ENCODER_POSITION = 8160;
static constexpr uint16_t PITCH_START_ENCODER_POSITION = 4100;
static constexpr float FEED_FORWARD_KP = 0.0f;  // TODO tune this value
static constexpr float FEED_FORWARD_MAX_OUTPUT = 20000.0f;

/// \todo TUNE the things
// World relative control
static constexpr aruwlib::algorithms::PidConfigStruct WR_YAW_PID_CONFIG =
    {4000.0f, 0.0f, 180.0f, 0.0f, 30000.0f, 1.0f, 20.0f, 1.0f, 10.0f};
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
static constexpr float FRICTION_WHEEL_TARGET_RPM = 7000;
}  // namespace launcher
}  // namespace hero_control::constants

#endif  // HERO_CONSTANTS_HPP_
