/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef BALSTD_CHASSIS_OUTPUT_HPP_
#define BALSTD_CHASSIS_OUTPUT_HPP_

#include "tap/algorithms/transforms/vector.hpp"

#include "balstd_leg.hpp"

namespace aruwsrc::balstd::chassis
{
struct BalstdChassisOutput
{
    tap::algorithms::transforms::Vector leftForce, rightForce;
    float leftTorque, rightTorque;

    BalstdChassisOutput(float flx, float fly, float frx, float fry, float tl, float tr)
        : leftForce(flx, fly, 0.0f),
          rightForce(frx, fry, 0.0f),
          leftTorque(tl),
          rightTorque(tr)
    {
    }

    BalstdChassisOutput(
        tap::algorithms::transforms::Vector leftForce,
        tap::algorithms::transforms::Vector rightForce,
        float tl,
        float tr)
        : leftForce(leftForce),
          rightForce(rightForce),
          leftTorque(tl),
          rightTorque(tr)
    {
    }
};

const BalstdChassisOutput ZERO_OUTPUT(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

}  // namespace aruwsrc::balstd::chassis

#endif  // BALSTD_CHASSIS_OUTPUT_HPP_