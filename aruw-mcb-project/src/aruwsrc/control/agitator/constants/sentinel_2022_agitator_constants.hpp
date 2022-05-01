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

#ifndef SENTINEL_2022_AGITATOR_CONSTANTS_HPP_
#define SENTINEL_2022_AGITATOR_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/dji_motor.hpp"

// Do not include this file directly: use agitator_constants.hpp instead.
#ifndef AGITATOR_CONSTANTS_HPP_
#error "Do not include this file directly! Use agitator_constants.hpp instead."
#endif

namespace aruwsrc::control::agitator::constants
{
static constexpr tap::algorithms::SmoothPidConfig AGITATOR_PID_CONFIG = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::motor::MotorId AGITATOR_MOTOR_ID = tap::motor::MOTOR7;

static constexpr tap::can::CanBus AGITATOR1_MOTOR_CAN_BUS = tap::can::CanBus::CAN_BUS1;
static constexpr tap::can::CanBus AGITATOR2_MOTOR_CAN_BUS = tap::can::CanBus::CAN_BUS2;
}  // namespace aruwsrc::control::agitator::constants

#endif  // SENTINEL_AGITATOR_CONSTANTS_HPP_
