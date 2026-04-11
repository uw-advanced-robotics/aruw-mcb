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
#ifndef BINNED_ENCODER_ALIGNMENT_HPP_
#define BINNED_ENCODER_ALIGNMENT_HPP_

#include <cmath>
#include <numeric>

#include "modm/math/geometry.hpp"

namespace aruwsrc::algorithms
{
namespace binned_encoder_alignment
{
struct Ratio
{
    uint32_t num;
    uint32_t den;

    constexpr Ratio(uint32_t n, uint32_t d)
    {
        const uint32_t common = std::gcd(n, d);
        num = n / common;
        den = d / common;
    }
};
template <uint32_t NUM, uint32_t DEN>
float calculatePosition(float localEncoderPosition, float globalEncoderPosition, float localOffset)
{
    const Ratio ratio(NUM, DEN);

    const float r = static_cast<float>(ratio.num) / ratio.den;

    const float position = localEncoderPosition * r + localOffset +
                           std::round(
                               (globalEncoderPosition - localEncoderPosition * r - localOffset) /
                               (M_TWOPI / static_cast<float>(ratio.den))) *
                               (M_TWOPI / static_cast<float>(ratio.den));

    return position;
}
}  // namespace binned_encoder_alignment
}  // namespace aruwsrc::algorithms

#endif  // BINNED_ENCODER_ALIGNMENT_HPP_