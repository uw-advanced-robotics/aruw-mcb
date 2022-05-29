/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include <gtest/gtest.h>

#include "aruwsrc/control/agitator/fire_rate_manager.hpp"

using namespace testing;
using namespace aruwsrc::control::agitator;
using namespace aruwsrc::control::governor;

TEST(FireRateManagerTest, getFireRatePeriod_setFireRate_not_called_returns_uint32_max)
{
    FireRateManager fireRateManager;
    EXPECT_NEAR(UINT32_MAX, fireRateManager.getFireRatePeriod(), 1E-5);
}

struct TestParams
{
    float fireRate;
    uint32_t expectedFireRatePeriod;
    FireRateReadinessState expectedReadinessState;
};

class FireRateManagerParameterizedTest : public TestWithParam<TestParams>
{
protected:
    FireRateManager fireRateManager;
};

TEST_P(FireRateManagerParameterizedTest, getFireRatePeriod)
{
    fireRateManager.setFireRate(GetParam().fireRate);

    EXPECT_NEAR(GetParam().expectedFireRatePeriod, fireRateManager.getFireRatePeriod(), 1E-5);
}

TEST_P(FireRateManagerParameterizedTest, getFireRateReadinessState)
{
    FireRateManager fireRateManager;

    fireRateManager.setFireRate(GetParam().fireRate);

    EXPECT_EQ(GetParam().expectedReadinessState, fireRateManager.getFireRateReadinessState());
}

static constexpr TestParams TEST_NEGATIVE_FIRE_RATE = {
    .fireRate = -10,
    .expectedFireRatePeriod = UINT32_MAX,
    .expectedReadinessState = FireRateReadinessState::NOT_READY,
};

static constexpr TestParams TEST_ZERO_FIRE_RATE = {
    .fireRate = 0,
    .expectedFireRatePeriod = UINT32_MAX,
    .expectedReadinessState = FireRateReadinessState::NOT_READY,
};

static constexpr TestParams TEST_POSITIVE_FIRE_RATE = {
    .fireRate = 100,
    .expectedFireRatePeriod = 10,
    .expectedReadinessState = FireRateReadinessState::READY_USE_RATE_LIMITING,
};

static constexpr TestParams TEST_FIRE_RATE_ROUNDS_DOWN_TO_NEAREST_INT = {
    .fireRate = 101,
    .expectedFireRatePeriod = 1000 / 100,
    .expectedReadinessState = FireRateReadinessState::READY_USE_RATE_LIMITING,
};

static constexpr TestParams TEST_FIRE_RATE_ROUNDS_UP_TO_NEAREST_INT = {
    .fireRate = 99,
    .expectedFireRatePeriod = 10,
    .expectedReadinessState = FireRateReadinessState::READY_USE_RATE_LIMITING,
};

INSTANTIATE_TEST_CASE_P(
    FireRateManagerTest,
    FireRateManagerParameterizedTest,
    Values(
        TEST_NEGATIVE_FIRE_RATE,
        TEST_ZERO_FIRE_RATE,
        TEST_POSITIVE_FIRE_RATE,
        TEST_FIRE_RATE_ROUNDS_DOWN_TO_NEAREST_INT,
        TEST_FIRE_RATE_ROUNDS_UP_TO_NEAREST_INT));
