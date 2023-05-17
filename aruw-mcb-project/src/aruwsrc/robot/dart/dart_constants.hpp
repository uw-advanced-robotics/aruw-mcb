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

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/turret/turret_motor_config.hpp"
#include "modm/math/geometry/angle.hpp"

namespace aruwsrc::control::turret
{
static constexpr tap::motor::MotorId PULL_MOTOR_ID =
    tap::motor::MOTOR6;  // TODO: update correct motor
static constexpr tap::motor::MotorId DEAD_MOTOR1 = tap::motor::MOTOR5;
static constexpr tap::motor::MotorId DEAD_MOTOR2 = tap::motor::MOTOR4;
static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

static constexpr float LOADER_HOOKING_POSITION = 0.0f;

static constexpr float TOP_LOADER_POSITION = 0.0f;
static constexpr float TOP_LOADER_HOOKING_POSITION = 0.0f;

static constexpr float MIDDLE_LOADER_POSITION = 0.0f;
static constexpr float MIDDLE_LOADER_HOOKING_POSITION = 0.0f;

static constexpr float BOTTOM_LOADER_POSITION = 0.0f;
static constexpr float BOTTOM_LOADER_HOOKING_POSITION = 0.0f;

static constexpr float PIVOT_OUTPOST_POSITION = 0.0f;
static constexpr float PIVOT_BASE_POSITION = 0.0f;


}  // namespace aruwsrc::control::turret
#endif
