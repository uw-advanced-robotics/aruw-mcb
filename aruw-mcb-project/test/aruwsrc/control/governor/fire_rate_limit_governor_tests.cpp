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

#include "aruwsrc/control/governor/fire_rate_limit_governor.hpp"
#include "aruwsrc/mock/fire_rate_reselection_manager_interface_mock.hpp"

using namespace testing;
using namespace aruwsrc::control::governor;
using namespace aruwsrc::control::agitator;

class FireRateLimitGovernorTest : public Test
{
protected:
    FireRateLimitGovernorTest() : governor(fireRateManager) {}

    NiceMock<aruwsrc::mock::FireRateReselectionManagerInterfaceMock> fireRateManager;
    FireRateLimitGovernor governor;
    tap::arch::clock::ClockStub clock;
};

TEST_F(FireRateLimitGovernorTest, isFinished_always_false) { EXPECT_FALSE(governor.isFinished()); }

namespace params
{
struct TestParams
{
    bool expectIsReady =
        true;  // isReady based on readiness state (i.e. ignoring the fire rate timer)
    FireRateReadinessState fireRateReadinessState = FireRateReadinessState::READY_USE_RATE_LIMITING;

    uint32_t fireRatePeriod = 10;

    uint32_t timeWhenInitializeCalled = 0;
    uint32_t timeWhenIsReadyCalled = 0;

    friend std::ostream& operator<<(std::ostream& os, const TestParams& p)
    {
        return os << "{" << p.expectIsReady << ", " << int(p.fireRateReadinessState) << ", "
                  << p.fireRatePeriod << ", " << p.timeWhenInitializeCalled << ", "
                  << p.timeWhenIsReadyCalled << "}";
    }
};
}  // namespace params

using namespace params;

class FireRateLimitGovernorParameterizedTest : public FireRateLimitGovernorTest,
                                               public WithParamInterface<TestParams>
{
    void SetUp() override
    {
        ON_CALL(fireRateManager, getFireRateReadinessState)
            .WillByDefault(Return(GetParam().fireRateReadinessState));

        ON_CALL(fireRateManager, getFireRatePeriod)
            .WillByDefault(Return(GetParam().fireRatePeriod));
    }
};

TEST_P(FireRateLimitGovernorParameterizedTest, isReady)
{
    EXPECT_EQ(GetParam().expectIsReady, governor.isReady());
}

TEST_P(FireRateLimitGovernorParameterizedTest, initialize)
{
    clock.time = GetParam().timeWhenInitializeCalled;

    governor.initialize();

    clock.time = GetParam().timeWhenIsReadyCalled;

    bool ready = GetParam().timeWhenIsReadyCalled - GetParam().timeWhenInitializeCalled >=
                 GetParam().fireRatePeriod;

    switch (GetParam().fireRateReadinessState)
    {
        case FireRateReadinessState::READY_IGNORE_RATE_LIMITING:
            EXPECT_EQ(true, governor.isReady());
            break;
        case FireRateReadinessState::READY_USE_RATE_LIMITING:
            EXPECT_EQ(ready, governor.isReady());
            break;
        case FireRateReadinessState::NOT_READY:
            EXPECT_EQ(false, governor.isReady());
            break;
    }
}

static constexpr TestParams TEST_FIRE_RATE_MANAGER_IGNORE_RATE_LIMITING = {
    .expectIsReady = true,
    .fireRateReadinessState = FireRateReadinessState::READY_IGNORE_RATE_LIMITING,
};

static constexpr TestParams TEST_FIRE_RATE_MANAGER_NOT_READY = {
    .expectIsReady = false,
    .fireRateReadinessState = FireRateReadinessState::NOT_READY,
};

static constexpr TestParams TEST_FIRE_RATE_MANAGER_USE_RATE_LIMITING = {
    .expectIsReady = true,
    .fireRateReadinessState = FireRateReadinessState::READY_USE_RATE_LIMITING,
    .fireRatePeriod = 0,
};

static constexpr TestParams TEST_FIRE_RATE_MANAGER_ISREADY_WITHIN_FIRE_RATE_PERIOD = {
    .expectIsReady = false,
    .fireRatePeriod = 10,
    .timeWhenInitializeCalled = 0,
    .timeWhenIsReadyCalled = 5,
};

static constexpr TestParams TEST_FIRE_RATE_MANAGER_ISREADY_OUTSIDE_OF_FIRE_RATE_PERIOD = {
    .expectIsReady = false,
    .fireRatePeriod = 10,
    .timeWhenInitializeCalled = 0,
    .timeWhenIsReadyCalled = 10,
};

static constexpr TestParams
    TEST_FIRE_RATE_MANAGER_ISREADY_OUTSIDE_OF_FIRE_RATE_PERIOD_NONZERO_INITIALIZE_TIME = {
        .expectIsReady = false,
        .fireRatePeriod = 10,
        .timeWhenInitializeCalled = 20,
        .timeWhenIsReadyCalled = 35,
};

INSTANTIATE_TEST_CASE_P(
    FireRateLimitGovernorTest,
    FireRateLimitGovernorParameterizedTest,
    Values(
        TEST_FIRE_RATE_MANAGER_IGNORE_RATE_LIMITING,
        TEST_FIRE_RATE_MANAGER_NOT_READY,
        TEST_FIRE_RATE_MANAGER_USE_RATE_LIMITING,

        TEST_FIRE_RATE_MANAGER_ISREADY_WITHIN_FIRE_RATE_PERIOD,
        TEST_FIRE_RATE_MANAGER_ISREADY_OUTSIDE_OF_FIRE_RATE_PERIOD,
        TEST_FIRE_RATE_MANAGER_ISREADY_OUTSIDE_OF_FIRE_RATE_PERIOD_NONZERO_INITIALIZE_TIME));
