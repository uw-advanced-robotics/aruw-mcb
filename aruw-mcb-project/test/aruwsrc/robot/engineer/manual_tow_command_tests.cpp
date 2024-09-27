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
#include "aruwsrc/robot/engineer/manual_tow_command.hpp"

using aruwsrc::engineer::ManualTowCommand;
using aruwsrc::mock::TowSubsystemMock;
using tap::gpio::Digital;
using namespace testing;

static constexpr Digital::OutputPin LEFT_TOW_PIN = Digital::OutputPin::E;
static constexpr Digital::OutputPin RIGHT_TOW_PIN = Digital::OutputPin::F;
static constexpr Digital::InputPin LEFT_TOW_LIMIT_SWITCH_PIN = Digital::InputPin::B;
static constexpr Digital::InputPin RIGHT_TOW_LIMIT_SWITCH_PIN = Digital::InputPin::C;

class ManualTowCommandTest : public Test
{
protected:
    ManualTowCommandTest()
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
    ManualTowCommand tc;
};

TEST_F(ManualTowCommandTest, isFinished_always_returns_false) { EXPECT_FALSE(tc.isFinished()); }

TEST_F(ManualTowCommandTest, initialize_sets_left_and_right_clamped_true)
{
    EXPECT_CALL(ts, setLeftClamped(true));
    EXPECT_CALL(ts, setRightClamped(true));

    tc.initialize();
}

TEST_F(ManualTowCommandTest, end_sets_left_and_right_clamped_false)
{
    EXPECT_CALL(ts, setLeftClamped(false)).Times(2);
    EXPECT_CALL(ts, setRightClamped(false)).Times(2);

    tc.end(false);
    tc.end(true);
}
