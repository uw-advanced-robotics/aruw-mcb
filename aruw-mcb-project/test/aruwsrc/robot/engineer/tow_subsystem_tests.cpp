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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "tap/drivers.hpp"

#include "aruwsrc/robot/engineer/tow_subsystem.hpp"

using aruwsrc::engineer::TowSubsystem;
using tap::gpio::Digital;
using namespace testing;

static constexpr Digital::OutputPin LEFT_TOW_PIN = Digital::OutputPin::E;
static constexpr Digital::OutputPin RIGHT_TOW_PIN = Digital::OutputPin::F;
static constexpr Digital::InputPin LEFT_TOW_LIMIT_SWITCH_PIN = Digital::InputPin::B;
static constexpr Digital::InputPin RIGHT_TOW_LIMIT_SWITCH_PIN = Digital::InputPin::C;

class TowSubsystemTest : public Test
{
protected:
    TowSubsystemTest()
        : ts(&drivers,
             LEFT_TOW_PIN,
             RIGHT_TOW_PIN,
             LEFT_TOW_LIMIT_SWITCH_PIN,
             RIGHT_TOW_LIMIT_SWITCH_PIN)
    {
    }

    tap::Drivers drivers;
    TowSubsystem ts;
};

// get*clamped

TEST_F(TowSubsystemTest, getLeftClamped_default_returns_false)
{
    EXPECT_FALSE(ts.getLeftClamped());
}

TEST_F(TowSubsystemTest, getRightClamped_default_returns_false)
{
    EXPECT_FALSE(ts.getRightClamped());
}

// get*LimitSwitchTriggered

TEST_F(TowSubsystemTest, getLeftLimitSwitchTriggered_returns_false_when_digital_pin_low)
{
    EXPECT_CALL(drivers.digital, read(LEFT_TOW_LIMIT_SWITCH_PIN)).WillOnce(Return(false));

    EXPECT_FALSE(ts.getLeftLimitSwitchTriggered());
}

TEST_F(TowSubsystemTest, getLeftLimitSwitchTriggered_returns_true_when_digital_pin_high)
{
    EXPECT_CALL(drivers.digital, read(LEFT_TOW_LIMIT_SWITCH_PIN)).WillOnce(Return(true));

    EXPECT_TRUE(ts.getLeftLimitSwitchTriggered());
}

TEST_F(TowSubsystemTest, getRightLimitSwitchTriggered_returns_false_when_digital_pin_low)
{
    EXPECT_CALL(drivers.digital, read(RIGHT_TOW_LIMIT_SWITCH_PIN)).WillOnce(Return(false));

    EXPECT_FALSE(ts.getRightLeftLimitSwitchTriggered());
}

TEST_F(TowSubsystemTest, getRightLimitSwitchTriggered_returns_true_when_digital_pin_high)
{
    EXPECT_CALL(drivers.digital, read(RIGHT_TOW_LIMIT_SWITCH_PIN)).WillOnce(Return(true));

    EXPECT_TRUE(ts.getRightLeftLimitSwitchTriggered());
}

// set*Clamped and get*Clamped

TEST_F(TowSubsystemTest, setLeftClamped_false_results_in_getLeftClamped_returning_false)
{
    EXPECT_CALL(drivers.digital, set);

    ts.setLeftClamped(false);
    EXPECT_FALSE(ts.getLeftClamped());
}

TEST_F(TowSubsystemTest, setLeftClamped_true_results_in_getLeftClamped_returning_true)
{
    EXPECT_CALL(drivers.digital, set);

    ts.setLeftClamped(true);
    EXPECT_TRUE(ts.getLeftClamped());
}

TEST_F(TowSubsystemTest, setRightClamped_false_results_in_getRightClamped_returning_false)
{
    EXPECT_CALL(drivers.digital, set);

    ts.setRightClamped(false);
    EXPECT_FALSE(ts.getRightClamped());
}

TEST_F(TowSubsystemTest, setRightClamped_true_results_in_getRightClamped_returning_true)
{
    EXPECT_CALL(drivers.digital, set);

    ts.setRightClamped(true);
    EXPECT_TRUE(ts.getRightClamped());
}
