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

namespace aruwsrc::control::turret::algorithms
{
/**
 * @param[in] cgX The center of gravity relative to the center of the turret's pitch pivot point
 *      in the X (forward/back) direction. The "X" direction lies along the plane that the turret
 *      is pointing. Units in millimeters. Positive is forward, negative is backwards.
 * @param[in] cgZ The center of gravity relative to the center of the turret's pitch pivot point,
 *      in the Z (up/down) direction. The "Z" direction lies perpendicular to the plane that the
 *      turret is pointing. Units in millimeters. Positive is upwards, negative is downwards.
 * @param[in] pitchAngleFromCenter The angle in degrees of the turret pitch, relative to the
 *      horizontal plane.
 * @param[in] gravityCompensationMotorOutputMax The maximum output that will be returned by this
 *      function. Should be equivalent to the output to offset gravity when the center of mass lies
 *      on the same xy-plane as the pivot (i.e.: when the turret's mass exerts the most torque about
 *      it's pivot).
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
}  // namespace aruwsrc::control::turret::algorithms

#endif  // GRAVITY_COMPENSATION_HPP_
