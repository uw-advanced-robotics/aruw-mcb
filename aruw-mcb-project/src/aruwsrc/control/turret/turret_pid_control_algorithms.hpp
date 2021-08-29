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

#ifndef TURRET_PID_CONTROL_ALGORITHMS_HPP_
#define TURRET_PID_CONTROL_ALGORITHMS_HPP_

#include "tap/algorithms/smooth_pid.hpp"

#include "turret_subsystem.hpp"

namespace aruwsrc::control::turret
{
void runSinglePidPitchChassisFrameController(
    const float dt,
    const float userInput,
    tap::algorithms::SmoothPid &pid,
    TurretSubsystem *turretSubsystem);
}

#endif  // TURRET_PID_CONTROL_ALGORITHMS_HPP_
