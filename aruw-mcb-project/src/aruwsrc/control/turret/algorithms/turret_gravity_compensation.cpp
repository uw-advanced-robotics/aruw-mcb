#include <iostream>

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

#include "tap/algorithms/math_user_utils.hpp"

#include "turret_gravity_compensation.hpp"

using namespace tap::algorithms;

namespace aruwsrc::control::turret
{
float computeGravitationalForceOffset(
    const float cgX,
    const float cgZ,
    const float pitchAngleFromCenter,
    const float gravityCompensatorMax)
{
    // If CG centered, no compensation necessary
    if (compareFloatClose(cgX, cgZ, 1E-5) && compareFloatClose(cgX, 0.0f, 1E-5))
    {
        return 0.0f;
    }

    float turretCGPolarTheta = 0.0f;
    if (cgX != 0.0f)
    {
        turretCGPolarTheta = (cgX > 0.0f) ? atanf(cgZ / cgX) : (atanf(cgZ / cgX) + M_PI);
    }
    else
    {
        turretCGPolarTheta = copysign(M_PI_2, cgZ);
    }

    return gravityCompensatorMax * cosf(turretCGPolarTheta + modm::toRadian(pitchAngleFromCenter));
}
}  // namespace aruwsrc::control::turret
