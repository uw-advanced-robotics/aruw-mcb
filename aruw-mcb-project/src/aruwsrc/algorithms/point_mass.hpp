/*
 * Copyright (c) 2026-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef POINT_MASS_HPP_
#define POINT_MASS_HPP_

#include "tap/algorithms/transforms/position.hpp"

namespace aruwsrc::algorithms
{
struct PointMass
{
    float mass;  // kg
    tap::algorithms::transforms::Position location;

    static PointMass merge(const PointMass& a, const PointMass& b)
    {
        return {
            .mass = a.mass + b.mass,
            .location = tap::algorithms::transforms::Position::interpolate(
                a.location,
                b.location,
                b.mass / (a.mass + b.mass))};
    }
};

}  // namespace aruwsrc::algorithms
#endif  // POINT_MASS_HPP_
