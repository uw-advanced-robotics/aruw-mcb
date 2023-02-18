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

#ifndef CHASSIS_CONSTANTS_HPP_
#define CHASSIS_CONSTANTS_HPP_

#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/util_macros.hpp"

#if defined(ALL_STANDARDS)
#include "standard_chassis_constants.hpp"
#elif defined(TARGET_HERO_CYCLONE)
#include "hero_chassis_constants.hpp"
#else  // by default use engineer constants (for robots that don't use them)
#include "engineer_chassis_constants.hpp"
#endif

#if defined(TARGET_SENTRY_BEEHIVE)
#include "sentry_chassis_constants.hpp"
#endif

namespace aruwsrc::chassis
{
// hardware constants, not specific to any particular chassis
static constexpr tap::motor::MotorId LEFT_FRONT_MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::motor::MotorId LEFT_BACK_MOTOR_ID = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId RIGHT_FRONT_MOTOR_ID = tap::motor::MOTOR3;
static constexpr tap::motor::MotorId RIGHT_BACK_MOTOR_ID = tap::motor::MOTOR4;

//TEMPORARY, for testing/getting it to build without mcb-lite
static constexpr tap::motor::MotorId LEFT_FRONT_AZIMUTH_MOTOR_ID = tap::motor::MOTOR5;
static constexpr tap::motor::MotorId LEFT_BACK_AZIMUTH_MOTOR_ID = tap::motor::MOTOR6;
static constexpr tap::motor::MotorId RIGHT_FRONT_AZIMUTH_MOTOR_ID = tap::motor::MOTOR7;
static constexpr tap::motor::MotorId RIGHT_BACK_AZIMUTH_MOTOR_ID = tap::motor::MOTOR8;

static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS2;
}  // namespace aruwsrc::chassis

#endif  // CHASSIS_CONSTANTS_HPP_
