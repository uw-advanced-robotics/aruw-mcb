/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef ENGINEER_GANTRY_CONSTANTS_HPP_
#define ENGINEER_GANTRY_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/gpio/digital.hpp"
#include "tap/motor/dji_motor.hpp"
namespace aruwsrc::engineer
{
static constexpr tap::can::CanBus CAN_BUS_GANTRY = tap::can::CanBus::CAN_BUS1;
static constexpr tap::motor::MotorId GANTRY_LIFT_LEFT_MOTOR_ID = tap::motor::MotorId::MOTOR1;
static constexpr tap::motor::MotorId GANTRY_LIFT_RIGHT_MOTOR_ID = tap::motor::MotorId::MOTOR2;
static constexpr tap::motor::MotorId GANTRY_EXTENSION_MOTOR_ID = tap::motor::MotorId::MOTOR3;

static constexpr float GANTRY_LIFT_POS_PID_KP = 2000.0f;
static constexpr float GANTRY_LIFT_POS_PID_KI = 30.0f;
static constexpr float GANTRY_LIFT_POS_PID_KD = 250.0f;
static constexpr float GANTRY_LIFT_POS_PID_MAX_ERROR_SUM = 0.0f;
static constexpr float GANTRY_LIFT_POS_PID_KS = 0.0;
static constexpr float GANTRY_LIFT_POS_MAX_OUTPUT = 6000.0f;

static constexpr float GANTRY_LIFT_LOWER_BOUND = 5.0f;
static constexpr float GANTRY_LIFT_UPPER_BOUND = 320.0f;
static constexpr float GANTRY_LIFT_RADIUS = 5 * 24 / M_TWOPI;  // 5mm per tooth, 24 teeth
static constexpr float GANTRY_LIFT_HOME = 0.0f;
static constexpr float GANTRY_LIFT_KS = 0.0f;
static constexpr float GANTRY_LIFT_EPSILON = 1.0f;

static constexpr float GANTRY_LIFT_BALANCE_PID_KP = 100.0f;  // todo
static constexpr float GANTRY_LIFT_BALANCE_PID_KI = 0.0f;
static constexpr float GANTRY_LIFT_BALANCE_PID_KD = 0.0f;
static constexpr float GANTRY_LIFT_BALANCE_PID_MAX_ERROR_SUM = 0.0f;
static constexpr float GANTRY_LIFT_BALANCE_PID_KS = 0.0;
static constexpr float GANTRY_LIFT_BALANCE_MAX_OUTPUT = 1000.0f;

static constexpr tap::gpio::Digital::InputPin GANTRY_LIFT_LIMIT_SWITCH_PIN =
    tap::gpio::Digital::InputPin::D;

static constexpr tap::algorithms::SmoothPidConfig GANTRY_LIFT_POS_CONFIG(
    GANTRY_LIFT_POS_PID_KP,
    GANTRY_LIFT_POS_PID_KI,
    GANTRY_LIFT_POS_PID_KD,
    GANTRY_LIFT_POS_PID_MAX_ERROR_SUM,
    GANTRY_LIFT_POS_MAX_OUTPUT);

static constexpr tap::algorithms::SmoothPidConfig GANTRY_LIFT_BALANCE_CONFIG(
    GANTRY_LIFT_BALANCE_PID_KP,
    GANTRY_LIFT_BALANCE_PID_KI,
    GANTRY_LIFT_BALANCE_PID_KD,
    GANTRY_LIFT_BALANCE_PID_MAX_ERROR_SUM,
    GANTRY_LIFT_BALANCE_MAX_OUTPUT);

static constexpr float GANTRY_EXTENSION_PID_KP = 300.0f;
static constexpr float GANTRY_EXTENSION_PID_KI = 0.0f;
static constexpr float GANTRY_EXTENSION_PID_KD = 40.0f;
static constexpr float GANTRY_EXTENSION_PID_MAX_ERROR_SUM = 0.0f;
static constexpr float GANTRY_EXTENSION_PID_KS = 0.0;
static constexpr float GANTRY_EXTENSION_MAX_OUTPUT = 2000.0f;

static constexpr float GANTRY_EXTENSION_LOWER_BOUND = 5.0f;
static constexpr float GANTRY_EXTENSION_UPPER_BOUND = 300.0f;
static constexpr float GANTRY_EXTENSION_RADIUS = 5 * 14 / M_TWOPI;  // 5mm per tooth, 14 teeth
static constexpr float GANTRY_EXTENSION_HOME = 0.0f;
static constexpr float GANTRY_EXTENSION_KS = 0.0f;
static constexpr float GANTRY_EXTENSION_EPSILON = 1.0f;

static constexpr tap::algorithms::SmoothPidConfig GANTRY_EXTENSION_CONFIG(
    GANTRY_EXTENSION_PID_KP,
    GANTRY_EXTENSION_PID_KI,
    GANTRY_EXTENSION_PID_KD,
    GANTRY_EXTENSION_PID_MAX_ERROR_SUM,
    GANTRY_EXTENSION_MAX_OUTPUT);

static constexpr tap::gpio::Digital::InputPin GANTRY_EXTENSION_LIMIT_SWITCH_PIN =
    tap::gpio::Digital::InputPin::T;  // TODO: Update to correct pin

static constexpr float GANTRY_LIFT_MOVE_SPEED = 0.6f;
static constexpr float GANTRY_EXTENSION_MOVE_SPEED = 0.6f;

}  // namespace aruwsrc::engineer
#endif  // ENGINEER_GANTRY_CONSTANTS_HPP_   `