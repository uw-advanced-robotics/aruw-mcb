/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "aruwsrc/control/cycle_state_command_mapping.hpp"

using namespace testing;
using namespace aruwsrc::control;
using namespace tap::communication::serial;

class TestCycleClass
{
public:
    enum SpecialState
    {
        STATE_1 = 0,
        STATE_2,
        STATE_3,
        MAX_STATES,
    };

    MOCK_METHOD(void, increment, (SpecialState), ());
};

class CycleStateCommandMappingTest : public Test
{
protected:
    CycleStateCommandMappingTest()
        : rms(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN),
          reverseRMS(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP),
          cmdMapping(
              &drivers,
              rms,
              TestCycleClass::STATE_1,
              &testCycleClass,
              &TestCycleClass::increment,
              reverseRMS)
    {
    }

    tap::Drivers drivers;
    tap::control::RemoteMapState rms;
    tap::control::RemoteMapState reverseRMS;
    tap::control::RemoteMapState nonMatchingRMS;
    TestCycleClass testCycleClass;
    CycleStateCommandMapping<
        TestCycleClass::SpecialState,
        TestCycleClass::MAX_STATES,
        TestCycleClass>
        cmdMapping;
};

TEST_F(CycleStateCommandMappingTest, executeCommandMapping_state_matches)
{
    InSequence seq;
    EXPECT_CALL(testCycleClass, increment(TestCycleClass::STATE_2));
    EXPECT_CALL(testCycleClass, increment(TestCycleClass::STATE_3));
    EXPECT_CALL(testCycleClass, increment(TestCycleClass::STATE_1));

    cmdMapping.executeCommandMapping(rms);
    cmdMapping.executeCommandMapping(nonMatchingRMS);
    cmdMapping.executeCommandMapping(rms);
    cmdMapping.executeCommandMapping(nonMatchingRMS);
    cmdMapping.executeCommandMapping(rms);
}

TEST_F(CycleStateCommandMappingTest, executeCommandMapping_state_doesnot_match_doesnot_update)
{
    EXPECT_CALL(testCycleClass, increment).Times(0);

    cmdMapping.executeCommandMapping(nonMatchingRMS);
    cmdMapping.executeCommandMapping(nonMatchingRMS);
    cmdMapping.executeCommandMapping(nonMatchingRMS);
}

TEST_F(
    CycleStateCommandMappingTest,
    executeCommandMapping_state_matches_multiple_times_in_row_updates_once)
{
    EXPECT_CALL(testCycleClass, increment);

    cmdMapping.executeCommandMapping(rms);
    cmdMapping.executeCommandMapping(rms);
    cmdMapping.executeCommandMapping(rms);
}

TEST_F(CycleStateCommandMappingTest, executeCommandMapping_reverse_state_matches)
{
    InSequence seq;
    EXPECT_CALL(testCycleClass, increment(TestCycleClass::STATE_3));
    EXPECT_CALL(testCycleClass, increment(TestCycleClass::STATE_2));
    EXPECT_CALL(testCycleClass, increment(TestCycleClass::STATE_1));
    EXPECT_CALL(testCycleClass, increment(TestCycleClass::STATE_3));

    cmdMapping.executeCommandMapping(reverseRMS);
    cmdMapping.executeCommandMapping(nonMatchingRMS);
    cmdMapping.executeCommandMapping(reverseRMS);
    cmdMapping.executeCommandMapping(nonMatchingRMS);
    cmdMapping.executeCommandMapping(reverseRMS);
    cmdMapping.executeCommandMapping(nonMatchingRMS);
    cmdMapping.executeCommandMapping(reverseRMS);
}
