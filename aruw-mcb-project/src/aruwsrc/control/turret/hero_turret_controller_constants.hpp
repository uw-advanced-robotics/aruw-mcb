/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef HERO_TURRET_CONTROLLER_CONSTANTS_HPP_
#define HERO_TURRET_CONTROLLER_CONSTANTS_HPP_

#include "tap/algorithms/fuzzy_pid.hpp"

#include "modm/math/interpolation/linear.hpp"

namespace aruwsrc::control::turret
{
namespace world_rel_turret_imu
{
static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_CONFIG = {
    .kp = 9.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 3'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr float KD_S = 0.01f;

/**
 * Columns correspond to error, rows to delta error
 */
static constexpr auto KD_FUZZY_RULE_TABLE = std::array<std::array<float, 7>, 7>({
    std::array<float, 7>({
        KD_S * tap::algorithms::FuzzyRuleTable::PS,
        KD_S* tap::algorithms::FuzzyRuleTable::NS,
        KD_S* tap::algorithms::FuzzyRuleTable::NB,
        KD_S* tap::algorithms::FuzzyRuleTable::NB,
        KD_S* tap::algorithms::FuzzyRuleTable::NB,
        KD_S* tap::algorithms::FuzzyRuleTable::NM,
        KD_S* tap::algorithms::FuzzyRuleTable::PS,
    }),
    std::array<float, 7>({
        KD_S * tap::algorithms::FuzzyRuleTable::PS,
        KD_S* tap::algorithms::FuzzyRuleTable::NS,
        KD_S* tap::algorithms::FuzzyRuleTable::NB,
        KD_S* tap::algorithms::FuzzyRuleTable::NM,
        KD_S* tap::algorithms::FuzzyRuleTable::NM,
        KD_S* tap::algorithms::FuzzyRuleTable::NS,
        KD_S* tap::algorithms::FuzzyRuleTable::ZO,
    }),
    std::array<float, 7>({
        KD_S * tap::algorithms::FuzzyRuleTable::ZO,
        KD_S* tap::algorithms::FuzzyRuleTable::NS,
        KD_S* tap::algorithms::FuzzyRuleTable::NM,
        KD_S* tap::algorithms::FuzzyRuleTable::NM,
        KD_S* tap::algorithms::FuzzyRuleTable::NS,
        KD_S* tap::algorithms::FuzzyRuleTable::NS,
        KD_S* tap::algorithms::FuzzyRuleTable::ZO,
    }),
    std::array<float, 7>({
        KD_S * tap::algorithms::FuzzyRuleTable::ZO,
        KD_S* tap::algorithms::FuzzyRuleTable::NS,
        KD_S* tap::algorithms::FuzzyRuleTable::NS,
        KD_S* tap::algorithms::FuzzyRuleTable::NS,
        KD_S* tap::algorithms::FuzzyRuleTable::NS,
        KD_S* tap::algorithms::FuzzyRuleTable::NS,
        KD_S* tap::algorithms::FuzzyRuleTable::ZO,
    }),
    std::array<float, 7>({
        KD_S * tap::algorithms::FuzzyRuleTable::ZO,
        KD_S* tap::algorithms::FuzzyRuleTable::ZO,
        KD_S* tap::algorithms::FuzzyRuleTable::ZO,
        KD_S* tap::algorithms::FuzzyRuleTable::ZO,
        KD_S* tap::algorithms::FuzzyRuleTable::ZO,
        KD_S* tap::algorithms::FuzzyRuleTable::ZO,
        KD_S* tap::algorithms::FuzzyRuleTable::ZO,
    }),
    std::array<float, 7>({
        KD_S * tap::algorithms::FuzzyRuleTable::PB,
        KD_S* tap::algorithms::FuzzyRuleTable::NS,
        KD_S* tap::algorithms::FuzzyRuleTable::PS,
        KD_S* tap::algorithms::FuzzyRuleTable::PS,
        KD_S* tap::algorithms::FuzzyRuleTable::PS,
        KD_S* tap::algorithms::FuzzyRuleTable::PS,
        KD_S* tap::algorithms::FuzzyRuleTable::PB,
    }),
    std::array<float, 7>({
        KD_S * tap::algorithms::FuzzyRuleTable::PB,
        KD_S* tap::algorithms::FuzzyRuleTable::PM,
        KD_S* tap::algorithms::FuzzyRuleTable::PM,
        KD_S* tap::algorithms::FuzzyRuleTable::PM,
        KD_S* tap::algorithms::FuzzyRuleTable::PS,
        KD_S* tap::algorithms::FuzzyRuleTable::PS,
        KD_S* tap::algorithms::FuzzyRuleTable::PB,
    }),
});

static tap::algorithms::FuzzyPidConfig YAW_FUZZY_POS_PID_CONFIG{
    .kpMin = YAW_POS_PID_CONFIG.kp,
    .kpMax = YAW_POS_PID_CONFIG.kp,
    .kiMin = YAW_POS_PID_CONFIG.ki,
    .kiMax = YAW_POS_PID_CONFIG.ki,
    .kdMin = 0.0f,
    .kdMax = 0.7f,
    .maxError = 180.0f,  ///< 180 degrees physical max angle error 
    .maxErrorDerivative = 720.0f,  ///< 2 rotations per second max speed of turret
    .kpTable = tap::algorithms::FuzzyRuleTable(),
    .kiTable = tap::algorithms::FuzzyRuleTable(),
    .kdTable = tap::algorithms::FuzzyRuleTable(KD_FUZZY_RULE_TABLE),
};

static constexpr tap::algorithms::SmoothPidConfig YAW_VEL_PID_CONFIG = {
    .kp = 900.0f,
    .ki = 5.0f,
    .kd = 0.0f,
    .maxICumulative = 2'000.0f,
    .maxOutput = 30000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_CONFIG = {
    .kp = 22.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 10'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_VEL_PID_CONFIG = {
    .kp = 750.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 30000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
};
}  // namespace world_rel_turret_imu

namespace world_rel_chassis_imu
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 2500.0f,
    .ki = 0.0f,
    .kd = 150.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 30000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 40.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};
}  // namespace world_rel_chassis_imu

namespace chassis_rel
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 2500.0f,
    .ki = 0.0f,
    .kd = 150.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 30000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 40.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    .kp = 4000.0f,
    .ki = 0.0f,
    .kd = 130.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 30000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 20.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 2.0f,
    .errDeadzone = 0.0f,
};
}  // namespace chassis_rel
}  // namespace aruwsrc::control::turret

#endif  // HERO_TURRET_CONTROLLER_CONSTANTS_HPP_
