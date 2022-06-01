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

#include "aruwsrc/control/agitator/fire_rate_timer.hpp"

using namespace testing;
using namespace aruwsrc::control::agitator;

class FireRateTimerTests : public Test
{
public:
    FireRateTimer fireRateTimer;
    tap::arch::clock::ClockStub clock;
};

TEST_F(FireRateTimerTests, isReadyToLaunchProjectile_initially_true)
{
    EXPECT_TRUE(fireRateTimer.isReadyToLaunchProjectile());
}

TEST_F(FireRateTimerTests, isReadyToLaunchProjectile_false_after_setProjectileLaunched)
{
    fireRateTimer.setProjectileLaunchPeriod(10);
    fireRateTimer.registerNewLaunchedProjectile();
    EXPECT_FALSE(fireRateTimer.isReadyToLaunchProjectile());
}

TEST_F(
    FireRateTimerTests,
    isReadyToLaunchProjectile_false_when_setProjectileLaunched_launch_period_not_complete)
{
    fireRateTimer.setProjectileLaunchPeriod(100);
    clock.time += 50;
    EXPECT_FALSE(fireRateTimer.isReadyToLaunchProjectile());
}

TEST_F(FireRateTimerTests, isReadyToLaunchProjectile_true_after_newLaunchPeriod_expired)
{
    fireRateTimer.setProjectileLaunchPeriod(100);
    clock.time += 100;
    EXPECT_TRUE(fireRateTimer.isReadyToLaunchProjectile());
}

TEST_F(FireRateTimerTests, multiple_calls_to_setProjectileLaunched_newLaunchPeriod_used_each_time)
{
    fireRateTimer.setProjectileLaunchPeriod(100);

    clock.time += 100;

    fireRateTimer.setProjectileLaunchPeriod(1000);
    fireRateTimer.registerNewLaunchedProjectile();

    clock.time += 100;

    EXPECT_FALSE(fireRateTimer.isReadyToLaunchProjectile());

    clock.time += 900;

    EXPECT_TRUE(fireRateTimer.isReadyToLaunchProjectile());
}
