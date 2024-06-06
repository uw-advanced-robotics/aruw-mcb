/*
 * Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef INTERPOLATE_HPP_
#define INTERPOLATE_HPP_

#include "tap/algorithms/transforms/position.hpp"

using namespace tap::algorithms::transforms;

namespace aruwsrc::algorithms
{

Position quadraticBezierInterpolation(
    const Position& a,
    const Position& b,
    const Position& c,
    float t)
{
    return Position::interpolate(Position::interpolate(a, b, t), Position::interpolate(b, c, t), t);
}

}  // namespace aruwsrc::algorithms

#endif  // INTERPOLATE_HPP_