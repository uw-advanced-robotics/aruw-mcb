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

#ifndef TURRET_GRAVITY_COMPENSATION_HPP_
#define TURRET_GRAVITY_COMPENSATION_HPP_

#include <cmath>
#include <cstdint>

#include "modm/math/geometry/angle.hpp"

namespace aruwsrc::control::turret
{
/**
 * @param[in] cgX The center of gravity relative to the center of the turret, in the X
 *      (forward/back) direction. Units in millimeters. Positive is forward, negative is backwards.
 * @param[in] cgZ The center of gravity relative to the center, in the Z (up/down) direction. Units
 *      in millimeters.
 * @param[in] pitchAngleFromCenter The angle in degrees of the turret pitch, relative to the
 *      horizontal plane.
 * @param[in] gravityCompensationMax The maximum gravity offset to be returned, the gravity is
 *      scaled by this value.
 * @return The gravitational force offset necessary to cancel out gravitational
 *      force of the turret, between [-gravityCompensatorMax, gravityCompensatorMax].
 *      The gravitational force offset is a function of the location of the CG and
 *      the current pitch angle.
 */
float computeGravitationalForceOffset(
    const float cgX,
    const float cgZ,
    const float pitchAngleFromCenter,
    const float gravityCompensatorMax);
}  // namespace aruwsrc::control::turret

#endif  // GRAVITY_COMPENSATION_HPP_
