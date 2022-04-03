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

#include "fuzzy_pid.hpp"

#include "tap/algorithms/math_user_utils.hpp"

using namespace tap::algorithms;

namespace tap::algorithms
{
modm::interpolation::Linear<modm::Pair<float, float>> FuzzyRuleTable::
    classificationLinearInterpolator(
        FuzzyRuleTable::FUZZY_CLASSIFICATION_TO_INDEX_LUT,
        MODM_ARRAY_SIZE(FuzzyRuleTable::FUZZY_CLASSIFICATION_TO_INDEX_LUT));

FuzzyPid::FuzzyPid(const FuzzyPidConfig &pidConfig, const SmoothPidConfig &smoothPidConfig)
    : SmoothPid(smoothPidConfig),
      config(pidConfig)
{
}

float FuzzyPid::runController(float error, float errorDerivative, float dt)
{
    FuzzyPid::udpatePidGains(
        limitVal(error / config.maxError, -1.0f, 1.0f),
        limitVal(errorDerivative / config.maxErrorDerivative, -1.0f, 1.0f));
    return SmoothPid::runController(error, errorDerivative, dt);
}

void FuzzyPid::udpatePidGains(float error, float errorDerivative)
{
    setP(limitVal(config.kpTable.getOutput(error, errorDerivative), config.kpMin, config.kpMax));
    setI(limitVal(config.kiTable.getOutput(error, errorDerivative), config.kiMin, config.kiMax));
    setD(limitVal(config.kdTable.getOutput(error, errorDerivative), config.kdMin, config.kdMax));
}

}  // namespace tap::algorithms
