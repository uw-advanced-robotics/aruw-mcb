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

#ifndef DART_TURRET_CONSTANTS_HPP_
#define DART_TURRET_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/dji_motor.hpp"

#include "../turret_motor_config.hpp"
#include "modm/math/geometry/angle.hpp"

// Do not include this file directly: use turret_constants.hpp instead.
#ifndef TURRET_CONSTANTS_HPP_
#error "Do not include this file directly! Use turret_controller_constants.hpp instead."
#endif

namespace aruwsrc::control::turret
{
static constexpr uint8_t NUM_TURRETS = 1;
static constexpr float TURRET_CG_X = -22.23;
static constexpr float TURRET_CG_Z = 9.96;
static constexpr float GRAVITY_COMPENSATION_SCALAR = 3'500.0f;

/*static constexpr float USER_YAW_INPUT_SCALAR = 0.02f;
static constexpr float USER_PITCH_INPUT_SCALAR = 0.02f;*/

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = M_PI_2,
    .startEncoderValue = 7500,
    .minAngle = modm::toRadian(65),
    .maxAngle = modm::toRadian(117),
    .limitMotorAngles = false,
};
static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = M_PI_2,
    .startEncoderValue = 1100,
    .minAngle = 0,
    .maxAngle = M_PI,
    .limitMotorAngles = false,
};
}  // namespace aruwsrc::control::turret
#endif
