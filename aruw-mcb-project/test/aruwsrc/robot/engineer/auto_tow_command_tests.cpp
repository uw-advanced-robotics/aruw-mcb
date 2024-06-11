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

#include "aruwsrc/mock/engineer/tow_subsystem_mock.hpp"
#include "aruwsrc/robot/engineer/auto_tow_command.hpp"

using aruwsrc::engineer::AutoTowCommand;
using aruwsrc::mock::TowSubsystemMock;
using tap::gpio::Digital;
using namespace testing;

static constexpr Digital::OutputPin LEFT_TOW_PIN = Digital::OutputPin::E;
static constexpr Digital::OutputPin RIGHT_TOW_PIN = Digital::OutputPin::F;
static constexpr Digital::InputPin LEFT_TOW_LIMIT_SWITCH_PIN = Digital::InputPin::B;
static constexpr Digital::InputPin RIGHT_TOW_LIMIT_SWITCH_PIN = Digital::InputPin::C;

class AutoTowCommandTest : public Test
{
protected:
    AutoTowCommandTest()
        : ts(&drivers,
             LEFT_TOW_PIN,
             RIGHT_TOW_PIN,
             LEFT_TOW_LIMIT_SWITCH_PIN,
             RIGHT_TOW_LIMIT_SWITCH_PIN),
          tc(&ts)
    {
    }

    tap::Drivers drivers;
    TowSubsystemMock ts;
    AutoTowCommand tc;
};

TEST_F(AutoTowCommandTest, execute_dont_trigger_auto_tow_when_neither_switches_enabled)
{
    EXPECT_CALL(ts, setLeftClamped).Times(0);
    EXPECT_CALL(ts, setRightClamped).Times(0);
    EXPECT_CALL(ts, getLeftClamped).WillOnce(Return(false));
    EXPECT_CALL(ts, getRightClamped).WillOnce(Return(false));
    EXPECT_CALL(ts, getLeftLimitSwitchTriggered).WillOnce(Return(false));
    EXPECT_CALL(ts, getRightLeftLimitSwitchTriggered).WillOnce(Return(false));

    tc.execute();
}

TEST_F(AutoTowCommandTest, execute_trigger_left_clamp_when_left_switch_enabled)
{
    EXPECT_CALL(ts, setLeftClamped(true));
    EXPECT_CALL(ts, setRightClamped).Times(0);
    EXPECT_CALL(ts, getLeftClamped).WillOnce(Return(false));
    EXPECT_CALL(ts, getRightClamped).WillOnce(Return(false));
    EXPECT_CALL(ts, getLeftLimitSwitchTriggered).WillOnce(Return(true));
    EXPECT_CALL(ts, getRightLeftLimitSwitchTriggered).WillOnce(Return(false));

    tc.execute();
}

TEST_F(AutoTowCommandTest, execute_trigger_right_clamp_when_right_switch_enabled)
{
    EXPECT_CALL(ts, setLeftClamped).Times(0);
    EXPECT_CALL(ts, setRightClamped(true));
    EXPECT_CALL(ts, getLeftClamped).WillOnce(Return(false));
    EXPECT_CALL(ts, getRightClamped).WillOnce(Return(false));
    EXPECT_CALL(ts, getLeftLimitSwitchTriggered).WillOnce(Return(false));
    EXPECT_CALL(ts, getRightLeftLimitSwitchTriggered).WillOnce(Return(true));

    tc.execute();
}

TEST_F(AutoTowCommandTest, execute_trigger_both_clamps_when_both_switches_enabled)
{
    EXPECT_CALL(ts, setLeftClamped(true));
    EXPECT_CALL(ts, setRightClamped(true));
    EXPECT_CALL(ts, getLeftClamped).WillOnce(Return(false));
    EXPECT_CALL(ts, getRightClamped).WillOnce(Return(false));
    EXPECT_CALL(ts, getLeftLimitSwitchTriggered).WillOnce(Return(true));
    EXPECT_CALL(ts, getRightLeftLimitSwitchTriggered).WillOnce(Return(true));

    tc.execute();
}

TEST_F(AutoTowCommandTest, execute_dont_trigger_both_clamps_when_both_clamps_already_enabled)
{
    EXPECT_CALL(ts, setLeftClamped).Times(0);
    EXPECT_CALL(ts, setRightClamped).Times(0);
    EXPECT_CALL(ts, getLeftClamped).WillOnce(Return(true));
    EXPECT_CALL(ts, getRightClamped).WillOnce(Return(true));

    tc.execute();
}

TEST_F(AutoTowCommandTest, end_always_sets_both_clamps_open)
{
    EXPECT_CALL(ts, setLeftClamped(false)).Times(2);
    EXPECT_CALL(ts, setRightClamped(false)).Times(2);

    tc.end(false);
    tc.end(true);
}

TEST_F(AutoTowCommandTest, isFinished_always_returns_false) { EXPECT_FALSE(tc.isFinished()); }
