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

#include "aruwsrc/control/agitator/fire_rate_timer_manager.hpp"

using namespace testing;
using namespace aruwsrc::control::agitator;

class FireRateTimerManagerTest : public Test
{
public:
    FireRateTimerManager fireRateTimerManager;
    tap::arch::clock::ClockStub clock;
};

TEST_F(FireRateTimerManagerTest, isReadyToLaunchProjectile_initially_true)
{
    EXPECT_TRUE(fireRateTimerManager.isReadyToLaunchProjectile());
}

TEST_F(FireRateTimerManagerTest, isReadyToLaunchProjectile_false_after_setProjectileLaunched)
{
    fireRateTimerManager.setProjectileLaunchPeriod(10);
    fireRateTimerManager.setProjectileLaunched();
    EXPECT_FALSE(fireRateTimerManager.isReadyToLaunchProjectile());
}

TEST_F(
    FireRateTimerManagerTest,
    isReadyToLaunchProjectile_false_when_setProjectileLaunched_launch_period_not_complete)
{
    fireRateTimerManager.setProjectileLaunchPeriod(100);
    clock.time += 50;
    EXPECT_FALSE(fireRateTimerManager.isReadyToLaunchProjectile());
}

TEST_F(FireRateTimerManagerTest, isReadyToLaunchProjectile_true_after_newLaunchPeriod_expired)
{
    fireRateTimerManager.setProjectileLaunchPeriod(100);
    clock.time += 100;
    EXPECT_TRUE(fireRateTimerManager.isReadyToLaunchProjectile());
}

TEST_F(
    FireRateTimerManagerTest,
    multiple_calls_to_setProjectileLaunched_newLaunchPeriod_used_each_time)
{
    fireRateTimerManager.setProjectileLaunchPeriod(100);

    clock.time += 100;

    fireRateTimerManager.setProjectileLaunchPeriod(1000);
    fireRateTimerManager.setProjectileLaunched();

    clock.time += 100;

    EXPECT_FALSE(fireRateTimerManager.isReadyToLaunchProjectile());

    clock.time += 900;

    EXPECT_TRUE(fireRateTimerManager.isReadyToLaunchProjectile());
}
