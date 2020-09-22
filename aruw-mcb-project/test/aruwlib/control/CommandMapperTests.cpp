/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include <aruwlib/Drivers.hpp>
#include <aruwlib/control/CommandMapper.hpp>
#include <aruwlib/control/CommandMapping.hpp>
#include <aruwlib/control/HoldCommandMapping.hpp>
#include <aruwlib/control/HoldRepeatCommandMapping.hpp>
#include <aruwlib/control/PressCommandMapping.hpp>
#include <aruwlib/control/RemoteMapState.hpp>
#include <aruwlib/control/ToggleCommandMapping.hpp>
#include <gtest/gtest.h>

#include "TestCommand.hpp"
#include "TestSubsystem.hpp"

using namespace aruwlib::control;
using aruwlib::Drivers;
using aruwlib::Remote;

TEST(CommandMapper, getSize_returns_number_of_valid_maps_added)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP);
    RemoteMapState ms3(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
}

TEST(CommandMapper, getAtIndex_nullptr_returned_if_greater_than_mapper_size_otherwise_valid)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);
    RemoteMapState ms(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);

    cm.addHoldMapping(ms, {&tc});
    EXPECT_NE(nullptr, cm.getAtIndex(0));
    EXPECT_EQ(nullptr, cm.getAtIndex(1));
    EXPECT_EQ(nullptr, cm.getAtIndex(2));
}

TEST(CommandMapper, addHoldMapping_successfully_adds_mapping)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);
    RemoteMapState ms(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    HoldCommandMapping holdCommandMappingForCompare(&drivers, {&tc}, ms);

    cm.addHoldMapping(ms, {&tc});
    const HoldCommandMapping *holdMappingPtr =
        dynamic_cast<const HoldCommandMapping *>(cm.getAtIndex(0));
    EXPECT_NE(nullptr, holdMappingPtr);
    EXPECT_EQ(holdCommandMappingForCompare, *holdMappingPtr);
}

TEST(
    CommandMapper,
    addHoldMapping_unsuccessfully_adds_mapping_if_RemoteMapState_matches_and_command_different)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc1(&ts);
    TestCommand tc2(&ts);
    CommandMapper cm(&drivers);
    RemoteMapState ms(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    EXPECT_CALL(drivers.errorController, addToErrorList);

    cm.addHoldMapping(ms, {&tc1});
    cm.addHoldMapping(ms, {&tc2});
    EXPECT_EQ(1, cm.getSize());
}

TEST(CommandMapper, addHoldRepeatMapping_successfully_adds_mapping)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);
    RemoteMapState ms(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    HoldRepeatCommandMapping mappingForCompare(&drivers, {&tc}, ms);

    cm.addHoldRepeatMapping(ms, {&tc});
    const HoldRepeatCommandMapping *holdRepeatMappingPtr =
        dynamic_cast<const HoldRepeatCommandMapping *>(cm.getAtIndex(0));
    EXPECT_NE(nullptr, holdRepeatMappingPtr);
    std::cout << holdRepeatMappingPtr << std::endl;
    EXPECT_EQ(mappingForCompare, *holdRepeatMappingPtr);
}

TEST(CommandMapper, addToggleMapping_successfully_adds_mapping)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);
    RemoteMapState ms(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    ToggleCommandMapping mappingForCompare(&drivers, {&tc}, ms);

    cm.addToggleMapping(ms, {&tc});
    const ToggleCommandMapping *toggleMappingPtr =
        dynamic_cast<const ToggleCommandMapping *>(cm.getAtIndex(0));
    EXPECT_NE(nullptr, toggleMappingPtr);
    EXPECT_EQ(mappingForCompare, *toggleMappingPtr);
}

TEST(CommandMapper, addPressMapping_successfully_adds_mapping)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);
    RemoteMapState ms(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    ToggleCommandMapping mappingForCompare(&drivers, {&tc}, ms);

    cm.addPressMapping(ms, {&tc});
    const PressCommandMapping *pressMappingPtr =
        dynamic_cast<const PressCommandMapping *>(cm.getAtIndex(0));
    EXPECT_NE(nullptr, pressMappingPtr);
    EXPECT_EQ(mappingForCompare, *pressMappingPtr);
}

TEST(
    CommandMapper,
    addHoldMapping_unsuccessfully_adds_mapping_if_RemoteMapState_matches_and_command_same)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);
    RemoteMapState ms(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);

    cm.addHoldMapping(ms, {&tc});
    cm.addHoldMapping(ms, {&tc});
    EXPECT_EQ(1, cm.getSize());
}
