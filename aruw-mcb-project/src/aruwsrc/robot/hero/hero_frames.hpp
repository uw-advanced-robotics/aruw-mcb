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

#ifndef HERO_FRAMES_HPP_
#define HERO_FRAMES_HPP_

#include "tap/algorithms/transforms/frame.hpp"
#include "tap/algorithms/transforms/transform.hpp"

using namespace tap::algorithms::transforms;

namespace aruwsrc::transforms
{

static const Frame turretMCB;

static const Frame turret;

static const Frame Chassis;

static const Frame ChassisMCB;

static const Transform<turretMCB, turret> turretMCBtoturret(0, 0, 0, M_PI_2, M_PI, 0.0f);
}  // namespace aruwsrc::transforms

#endif  // HERO_FRAMES_HPP_
