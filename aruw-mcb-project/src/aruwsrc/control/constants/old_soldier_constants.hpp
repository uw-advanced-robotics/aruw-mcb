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

#ifndef OLD_SOLDIER_CONSTANTS_HPP_
#define OLD_SOLDIER_CONSTANTS_HPP_

#include "aruwlib/communication/can/can_bus.hpp"
#include "aruwlib/motor/dji_motor.hpp"

namespace old_soldier_control::constants
{
namespace motor
{
// CAN 1
static constexpr aruwlib::motor::MotorId YAW_MOTOR_ID = aruwlib::motor::MOTOR5;
static constexpr aruwlib::motor::MotorId PITCH_MOTOR_ID = aruwlib::motor::MOTOR6;
static constexpr aruwlib::motor::MotorId AGITATOR_MOTOR_ID = aruwlib::motor::MOTOR7;

// CAN 2
static constexpr aruwlib::motor::MotorId RIGHT_FRONT_MOTOR_ID = aruwlib::motor::MOTOR1;
static constexpr aruwlib::motor::MotorId LEFT_FRONT_MOTOR_ID = aruwlib::motor::MOTOR2;
static constexpr aruwlib::motor::MotorId LEFT_BACK_MOTOR_ID = aruwlib::motor::MOTOR3;
static constexpr aruwlib::motor::MotorId RIGHT_BACK_MOTOR_ID = aruwlib::motor::MOTOR4;
}  // namespace motor

namespace can
{
static constexpr aruwlib::can::CanBus TURRET_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;
static constexpr aruwlib::can::CanBus AGITATOR_MOTOR_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;
static constexpr aruwlib::can::CanBus CHASSIS_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;
}  // namespace can

namespace agitator
{
// position PID terms
// PID terms for soldier
static constexpr float PID_17MM_P = 170000.0f;
static constexpr float PID_17MM_I = 0.0f;
static constexpr float PID_17MM_D = 80.0f;
static constexpr float PID_17MM_MAX_ERR_SUM = 0.0f;
static constexpr float PID_17MM_MAX_OUT = 16000.0f;

static constexpr bool AGITATOR_INVERTED = false;
}  // namespace agitator

namespace chassis
{
static constexpr int MAX_WHEEL_SPEED_SINGLE_MOTOR = 7000;
static constexpr float VELOCITY_PID_KP = 20.0f;
static constexpr float VELOCITY_PID_KI = 0.0f;
static constexpr float VELOCITY_PID_KD = 0.0f;
static constexpr float VELOCITY_PID_MAX_ERROR_SUM = 0.0f;

static constexpr float VELOCITY_PID_MAX_OUTPUT = 16000.0f;

static constexpr float CHASSIS_REVOLVE_PID_MAX_P = MAX_WHEEL_SPEED_SINGLE_MOTOR;
static constexpr float CHASSIS_REVOLVE_PID_KD = 500.0f;
static constexpr float CHASSIS_REVOLVE_PID_MAX_D = 0.0f;
static constexpr int CHASSIS_REVOLVE_PID_MIN_ERROR_ROTATION_D = 0;
static constexpr float CHASSIS_REVOLVE_PID_MAX_OUTPUT = 5000.0f;

static constexpr float WHEEL_RADIUS = 0.076;
static constexpr float WIDTH_BETWEEN_WHEELS_Y = 0.366f;
static constexpr float WIDTH_BETWEEN_WHEELS_X = 0.366f;
static constexpr float GIMBAL_X_OFFSET = 0.0f;
static constexpr float GIMBAL_Y_OFFSET = 0.0f;
static constexpr float CHASSIS_GEARBOX_RATIO = (1.0f / 19.0f);

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

// CV control
static constexpr float YAW_CV_P = 4500.0f;
static constexpr float YAW_CV_I = 0.0f;
static constexpr float YAW_CV_D = 140.0f;
static constexpr float YAW_CV_MAX_ERROR_SUM = 0.0f;
static constexpr float YAW_CV_MAX_OUTPUT = 32000.0f;
static constexpr float YAW_CV_Q_DERIVATIVE_KALMAN = 1.0f;
static constexpr float YAW_CV_R_DERIVATIVE_KALMAN = 20.0f;
static constexpr float YAW_CV_Q_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float YAW_CV_R_PROPORTIONAL_KALMAN = 0.0f;
static constexpr float PITCH_CV_P = 3500.0f;
static constexpr float PITCH_CV_I = 0.0f;
static constexpr float PITCH_CV_D = 80.0f;
static constexpr float PITCH_CV_MAX_ERROR_SUM = 0.0f;
static constexpr float PITCH_CV_MAX_OUTPUT = 32000.0f;
static constexpr float PITCH_CV_Q_DERIVATIVE_KALMAN = 1.5f;
static constexpr float PITCH_CV_R_DERIVATIVE_KALMAN = 20.0f;
static constexpr float PITCH_CV_Q_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float PITCH_CV_R_PROPORTIONAL_KALMAN = 2.0f;

// World relative control
static constexpr float YAW_WR_P = 4000.0f;
static constexpr float YAW_WR_I = 0.0f;
static constexpr float YAW_WR_D = 180.0f;
static constexpr float YAW_WR_MAX_ERROR_SUM = 0.0f;
static constexpr float YAW_WR_MAX_OUTPUT = 30000.0f;
static constexpr float YAW_WR_Q_DERIVATIVE_KALMAN = 1.0f;
static constexpr float YAW_WR_R_DERIVATIVE_KALMAN = 20.0f;
static constexpr float YAW_WR_Q_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float YAW_WR_R_PROPORTIONAL_KALMAN = 10.0f;
static constexpr float PITCH_WR_P = 4500.0f;
static constexpr float PITCH_WR_I = 0.0f;
static constexpr float PITCH_WR_D = 90.0f;
static constexpr float PITCH_WR_MAX_ERROR_SUM = 0.0f;
static constexpr float PITCH_WR_MAX_OUTPUT = 30000.0f;
static constexpr float PITCH_WR_Q_DERIVATIVE_KALMAN = 1.5f;
static constexpr float PITCH_WR_R_DERIVATIVE_KALMAN = 20.0f;
static constexpr float PITCH_WR_Q_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float PITCH_WR_R_PROPORTIONAL_KALMAN = 2.0f;

static constexpr float USER_YAW_INPUT_SCALAR = 1.0f;
static constexpr float USER_PITCH_INPUT_SCALAR = 0.5f;

static constexpr float PITCH_GRAVITY_COMPENSATION_KP = 4000.0f;
}  // namespace turret
}  // namespace aruwsrc

#endif  // OLD_SOLDIER_CONSTANTS_HPP_
