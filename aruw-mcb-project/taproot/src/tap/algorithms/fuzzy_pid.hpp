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

#ifndef TAPROOT_FUZZY_PID_HPP_
#define TAPROOT_FUZZY_PID_HPP_

#include <array>
#include <cstdint>

#include "tap/algorithms/extended_kalman.hpp"

#include "modm/math/interpolation/linear.hpp"

#include "smooth_pid.hpp"

namespace tap
{
namespace algorithms
{
class FuzzyRuleTable
{
public:
    static constexpr float NB = -1;
    static constexpr float NM = .5;
    static constexpr float NS = .1;
    static constexpr float ZO = 0;
    static constexpr float PS = .1;
    static constexpr float PM = .5;
    static constexpr float PB = 1;

    static constexpr modm::Pair<float, float> FUZZY_CLASSIFICATION_TO_INDEX_LUT[] = {
        {NB, 0},
        {NM, 2},
        {NS, 3},
        {ZO, 4},
        {PS, 5},
        {PM, 6},
        {PB, 8},
    };

    FuzzyRuleTable() : alwaysZero(true), ruleTable{} {}

    explicit FuzzyRuleTable(const std::array<std::array<float, 7>, 7> &ruleTable)
        : alwaysZero(false),
          ruleTable(ruleTable)
    {
    }

    float getOutput(float error, float errorDerivative)
    {
        if (alwaysZero)
        {
            return 0;
        }
        else
        {
            int errIndex = getIndex(error);
            int errDerivativeIndex = getIndex(errorDerivative);
            return ruleTable[errDerivativeIndex][errIndex];
        }
    }

private:
    static modm::interpolation::Linear<modm::Pair<float, float>> classificationLinearInterpolator;

    bool alwaysZero;
    std::array<std::array<float, 7>, 7> ruleTable;

    static inline int getIndex(float val)
    {
        return round(classificationLinearInterpolator.interpolate(val));
    }
};

struct FuzzyPidConfig
{
    float kpMin = 0.0f;
    float kpMax = 0.0f;
    float kiMin = 0.0f;
    float kiMax = 0.0f;
    float kdMin = 0.0f;
    float kdMax = 0.0f;
    float maxError = 0.0f;
    float maxErrorDerivative = 0.0f;
    FuzzyRuleTable kpTable;
    FuzzyRuleTable kiTable;
    FuzzyRuleTable kdTable;
};

/**
 * Fuzzy PID controller.
 */
class FuzzyPid : public SmoothPid
{
public:
    FuzzyPid(const FuzzyPidConfig &pidConfig, const SmoothPidConfig &smoothPidConfig);

    float runController(float error, float errorDerivative, float dt) override;

private:
    // gains and constants, to be set by the user
    FuzzyPidConfig config;

    void udpatePidGains(float error, float errorDerivative);
};

}  // namespace algorithms

}  // namespace tap

#endif  // TAPROOT_FUZZY_PID_HPP_
