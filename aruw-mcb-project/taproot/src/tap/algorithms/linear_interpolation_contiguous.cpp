/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "linear_interpolation_contiguous.hpp"

namespace tap::algorithms
{
LinearInterpolationContiguous::LinearInterpolationContiguous(float lowerBound, float upperBound)
    : lastUpdateCallTime(0),
      previousValue(0.0f, lowerBound, upperBound),
      slope(0.0f)
{
}

void LinearInterpolationContiguous::update(float newValue, uint32_t currTime)
{
    if (currTime == lastUpdateCallTime)
    {
        return;
    }
    slope = (-previousValue.difference(newValue)) / (currTime - lastUpdateCallTime);
    previousValue.setValue(newValue);
    lastUpdateCallTime = currTime;
}

float LinearInterpolationContiguous::getInterpolatedValue(uint32_t currTime)
{
    return ContiguousFloat(
               slope * static_cast<float>(currTime - lastUpdateCallTime) + previousValue.getValue(),
               previousValue.getLowerBound(),
               previousValue.getUpperBound())
        .getValue();
}

void LinearInterpolationContiguous::reset(float initialValue, uint32_t initialTime)
{
    previousValue.setValue(initialValue);
    lastUpdateCallTime = initialTime;
    slope = 0.0f;
}
}  // namespace tap
