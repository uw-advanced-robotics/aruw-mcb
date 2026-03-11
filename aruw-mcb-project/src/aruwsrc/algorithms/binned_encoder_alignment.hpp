/*
 * Copyright (c) 2020-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include <cmath>

#include "modm/math/geometry.hpp"

namespace algorithms::binnedAlignment
{
float calculatePosition(
    float localEncoderPosition,
    float globalEncoderPosition,
    float gearRatioNumerator,
    float gearRatioDenominator,
    float localOffset)

{
    const float r = gearRatioNumerator / gearRatioDenominator;

    const float position = localEncoderPosition * r + localOffset +
                           std::round(
                               (globalEncoderPosition - localEncoderPosition * r - localOffset) /
                               (M_TWOPI / gearRatioDenominator)) *
                               (M_TWOPI / gearRatioDenominator);

    return position;
}
}  // namespace algorithms::binnedAlignment