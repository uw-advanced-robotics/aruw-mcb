/*
 * Copyright (c) 2023-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef LAUNCHER_TARGET_CONSTANTS_HPP_
#define LAUNCHER_TARGET_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/unjam_integral_command.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/agitator/unjam_spoke_agitator_command.hpp"
#include "aruwsrc/control/agitator/velocity_agitator_subsystem_config.hpp"
#include "modm/math/geometry/angle.hpp"

namespace aruwsrc::launcher_target::constants
{
// velocity pid terms
static constexpr tap::algorithms::SmoothPidConfig m2006VelocityPidConfig = {
    .kp = 50.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C610};

static constexpr tap::algorithms::SmoothPidConfig Ak809VelocityPidConfig =
    {.kp = 50.0f, .ki = 0.0f, .kd = 0.0f, .maxICumulative = 0.0f, .maxOutput = 16000.0f};

static constexpr float WHEEL_DIAMETER = 0.030f;
static constexpr float TARGET_MOVEMENT_SPEED =
    1.0f / WHEEL_DIAMETER / M_PI * 60.0f;  // 1 m/s / 0.03pi pulley circumference * 60sec/min
static constexpr uint32_t TERMINAL_MOVING_TARGET_DELAY = 1200;
static constexpr uint32_t TARGET_TIMEOUT = 600;
static constexpr float TARGET_TRAVEL_DISTANCE = .360f;
}  // namespace aruwsrc::launcher_target::constants

#endif  // LAUNCHER_TARGET_CONSTANTS_HPP_
