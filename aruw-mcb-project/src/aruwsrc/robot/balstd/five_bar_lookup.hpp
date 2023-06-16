/*
 * Copyright (c) 2023-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef FIVE_BAR_LOOKUP_HPP_
#define FIVE_BAR_LOOKUP_HPP_
#include <array>

#include "tap/algorithms/cmsis_mat.hpp"

using namespace tap::algorithms;

namespace aruwsrc::chassis
{
static constexpr float LQR_K1 = -2.2361;
static constexpr float LQR_K5 = -1.7321;
static constexpr float LQR_K6 = -0.7905;

static constexpr modm::Pair<float, float> HEIGHT_TO_LQR_LOOKUP_K2[] = {
    {0.35, -4.7942},
    {0.15, -4.7735},
};

static constexpr modm::Pair<float, float> HEIGHT_TO_LQR_LOOKUP_K3[] = {
    {0.35, -25.3590},
    {0.15, -25.1428},
};

static constexpr modm::Pair<float, float> HEIGHT_TO_LQR_LOOKUP_K4[] = {
    {0.35, -3.4459},
    {0.15, -3.0319},
};

static modm::interpolation::Linear<modm::Pair<float, float>> HEIGHT_TO_LQR_K2_INTERPOLATOR(
    HEIGHT_TO_LQR_LOOKUP_K2,
    MODM_ARRAY_SIZE(HEIGHT_TO_LQR_LOOKUP_K2));
static modm::interpolation::Linear<modm::Pair<float, float>> HEIGHT_TO_LQR_K3_INTERPOLATOR(
    HEIGHT_TO_LQR_LOOKUP_K3,
    MODM_ARRAY_SIZE(HEIGHT_TO_LQR_LOOKUP_K3));
static modm::interpolation::Linear<modm::Pair<float, float>> HEIGHT_TO_LQR_K4_INTERPOLATOR(
    HEIGHT_TO_LQR_LOOKUP_K4,
    MODM_ARRAY_SIZE(HEIGHT_TO_LQR_LOOKUP_K4));

static const float VMC_JACOBIAN_DATA[4] = {
    -0.0905158857883341,
    0.385086352953914,
    0.0905158857883341,
    0.385086352953913};
static const float VMC_JACOBIAN_INV_DATA[4] = {-5.5239, 5.5239, 1.2984, 1.2984};

static const CMSISMat<2, 2> VMC_JACOBIAN(VMC_JACOBIAN_DATA);

static const CMSISMat<2, 2> VMC_JACOBIAN_INV(VMC_JACOBIAN_INV_DATA);

static const float LQR_DATA_SLOPE[12] = {
    -99.27324,
    -21.87294,
    -3.30078,
    -8.15020,
    -67.95185,
    -6.56304,
    9.61597,
    4.24791,
    -15.19573,
    -25.60578,
    72.76464,
    8.94952,
};

static const CMSISMat<2, 6> LQR_K_SLOPE(LQR_DATA_SLOPE);

static const float LQR_DATA_YINT[12] = {
    -12.74785,
    -0.47628,
    -5.71773,
    -10.08875,
    43.29004,
    4.86263,
    23.51436,
    2.97989,
    9.03230,
    15.91147,
    76.31916,
    4.22845,
};

static const CMSISMat<2, 6> LQR_K_YINT(LQR_DATA_YINT);

}  // namespace aruwsrc::chassis

#endif