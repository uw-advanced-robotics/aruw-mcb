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

#ifndef DRONE_TURRET_CONTROLLER_CONSTANTS_HPP_
#define DRONE_TURRET_CONTROLLER_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/dji_motor.hpp"

namespace aruwsrc::control::turret
{
static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;
static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR6;
static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR5;

static constexpr float YAW_START_ANGLE = 90.0f;
static constexpr float YAW_MIN_ANGLE = 0.0f;
static constexpr float YAW_MAX_ANGLE = 180.0f;
static constexpr float PITCH_START_ANGLE = 90.0f;
static constexpr float PITCH_MIN_ANGLE = 0.0f;
static constexpr float PITCH_MAX_ANGLE = 180.0f;

static constexpr uint16_t YAW_START_ENCODER_POSITION = 0;
static constexpr uint16_t PITCH_START_ENCODER_POSITION = 0;

static constexpr float TURRET_CG_X = 0;
static constexpr float TURRET_CG_Z = 0;
static constexpr float GRAVITY_COMPENSATION_SCALAR = 1.0f;
}  // namespace aruwsrc::control::turret

#endif  // DRONE_TURRET_CONTROLLER_CONSTANTS_HPP_
