/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "linear_interpolation.hpp"

#include <aruwlib/architecture/clock.hpp>

namespace aruwlib
{
namespace algorithms
{
LinearInterpolation::LinearInterpolation() : lastUpdateCallTime(0), previousValue(0.0f), slope(0.0f)
{
}

void LinearInterpolation::update(float newValue)
{
    uint32_t currTime = aruwlib::arch::clock::getTimeMilliseconds();
    slope = (newValue - previousValue) / (currTime - lastUpdateCallTime);
    previousValue = newValue;
    lastUpdateCallTime = currTime;
}

float LinearInterpolation::getInterpolatedValue(uint32_t currTime)
{
    return slope * static_cast<float>(currTime - lastUpdateCallTime) + previousValue;
}
}  // namespace algorithms

}  // namespace aruwlib
