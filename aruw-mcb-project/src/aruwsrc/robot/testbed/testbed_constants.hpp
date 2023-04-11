/*
 * Copyright (c) 2023-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TESTBED_CONSTANTS_HPP_
#define TESTBED_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"

#include "aruwsrc/control/turret/turret_motor_config.hpp"

namespace aruwsrc::control::turret
{
static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 0,
    .minAngle = 0,
    .maxAngle = 0,
    .limitMotorAngles = true,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 0,
    .minAngle = 0,
    .maxAngle = 0,
    .limitMotorAngles = true,
};
static constexpr uint8_t NUM_TURRETS = 1;

static constexpr float USER_YAW_INPUT_SCALAR = 0.02f;
static constexpr float USER_PITCH_INPUT_SCALAR = 0.02f;

static constexpr float TURRET_CG_X = 30.17;
static constexpr float TURRET_CG_Z = 34.02;
static constexpr float GRAVITY_COMPENSATION_SCALAR = 0;
}  // namespace aruwsrc::control::turret

const static tap::algorithms::SmoothPidConfig TURRET_PID_CONFIG = {
    .kp = 100,
    .ki = 0,
    .kd = 0,
};



#endif