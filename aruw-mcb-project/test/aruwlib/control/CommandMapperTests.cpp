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

#include <aruwlib/Drivers.hpp>
#include <aruwlib/control/CommandMapper.hpp>
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
    TestCommand tc1(&ts);
    TestCommand tc2(&ts);
    TestCommand tc3(&ts);
    CommandMapper cm(&drivers);

    HoldCommandMapping hcm1(
        &drivers,
        {&tc1},
        RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));
    HoldCommandMapping hcm2(
        &drivers,
        {&tc2},
        RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));
    HoldCommandMapping hcm3(
        &drivers,
        {&tc3},
        RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

    EXPECT_EQ(0, cm.getSize());
    cm.addMap(&hcm1);
    cm.addMap(&hcm2);
    EXPECT_EQ(2, cm.getSize());
    cm.addMap(&hcm3);
    EXPECT_EQ(3, cm.getSize());
}

TEST(CommandMapper, getAtIndex_nullptr_returned_if_greater_than_mapper_size)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);
    HoldCommandMapping hcm(
        &drivers,
        {&tc},
        RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));

    cm.addMap(&hcm);
    EXPECT_NE(nullptr, cm.getAtIndex(0));
    EXPECT_EQ(nullptr, cm.getAtIndex(1));
    EXPECT_EQ(nullptr, cm.getAtIndex(2));
}

TEST(CommandMapper, getAtIndex_returns_correct_CommandMapping_if_index_valid)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc1(&ts);
    TestCommand tc2(&ts);
    TestCommand tc3(&ts);
    CommandMapper cm(&drivers);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID);
    RemoteMapState ms3(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP);
    HoldCommandMapping hcm1(
        &drivers,
        {&tc1},
        RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));
    HoldCommandMapping hcm2(
        &drivers,
        {&tc2},
        RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));
    HoldCommandMapping hcm3(
        &drivers,
        {&tc3},
        RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));
    cm.addMap(&hcm1);
    cm.addMap(&hcm2);
    cm.addMap(&hcm3);

    const CommandMapping *mapping = cm.getAtIndex(0);
    EXPECT_EQ(ms1, mapping->getAssociatedRemoteMapState());
    EXPECT_EQ(&tc1, mapping->getAssociatedCommands()[0]);
    mapping = cm.getAtIndex(1);
    EXPECT_EQ(ms2, mapping->getAssociatedRemoteMapState());
    EXPECT_EQ(&tc2, mapping->getAssociatedCommands()[0]);
    mapping = cm.getAtIndex(2);
    EXPECT_EQ(ms3, mapping->getAssociatedRemoteMapState());
    EXPECT_EQ(&tc3, mapping->getAssociatedCommands()[0]);
}

TEST(CommandMapper, addHoldMapping_successfully_adds_mapping_normal_case)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);
    RemoteMapState ms(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    HoldCommandMapping holdCommandMappingForCompare(&drivers, {&tc}, ms);
    HoldCommandMapping hm(&drivers, {&tc}, ms);

    cm.addMap(&hm);
    const HoldCommandMapping *holdMappingPtr =
        dynamic_cast<const HoldCommandMapping *>(cm.getAtIndex(0));
    EXPECT_NE(nullptr, holdMappingPtr);
    EXPECT_EQ(holdCommandMappingForCompare, *holdMappingPtr);
}

TEST(CommandMapper, addHoldMapping_fails_to_add_mapping_if_identical_mapping_already_added)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc1(&ts);
    TestCommand tc2(&ts);
    CommandMapper cm(&drivers);
    RemoteMapState ms(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID);
    EXPECT_CALL(drivers.errorController, addToErrorList);
    HoldCommandMapping hcm1(&drivers, {&tc1}, ms);
    HoldCommandMapping hcm2(&drivers, {&tc2}, ms);

    cm.addMap(&hcm1);
    EXPECT_EQ(1, cm.getSize());
    cm.addMap(&hcm2);
    EXPECT_EQ(1, cm.getSize());
    EXPECT_EQ(&tc1, cm.getAtIndex(0)->getAssociatedCommands()[0]);
}

TEST(CommandMapper, addHoldMapping_fails_to_add_mapping_if_RemoteMapState_matches_and_command_same)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);
    RemoteMapState ms(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    EXPECT_CALL(drivers.errorController, addToErrorList);
    HoldCommandMapping hcm1(&drivers, {&tc}, ms);
    HoldCommandMapping hcm2(&drivers, {&tc}, ms);

    cm.addMap(&hcm1);
    cm.addMap(&hcm2);
    EXPECT_EQ(1, cm.getSize());
}

TEST(CommandMapper, addHoldRepeatMapping_successfully_adds_mapping_normal_case)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);
    RemoteMapState ms(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    HoldRepeatCommandMapping mappingForCompare(&drivers, {&tc}, ms);

    cm.addMap(&mappingForCompare);
    const HoldRepeatCommandMapping *holdRepeatMappingPtr =
        dynamic_cast<const HoldRepeatCommandMapping *>(cm.getAtIndex(0));
    EXPECT_NE(nullptr, holdRepeatMappingPtr);
    EXPECT_EQ(mappingForCompare, *holdRepeatMappingPtr);
}

TEST(CommandMapper, addToggleMapping_successfully_adds_mapping_normal_case)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);
    RemoteMapState ms(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    ToggleCommandMapping mappingForCompare(&drivers, {&tc}, ms);

    cm.addMap(&mappingForCompare);
    const ToggleCommandMapping *toggleMappingPtr =
        dynamic_cast<const ToggleCommandMapping *>(cm.getAtIndex(0));
    EXPECT_NE(nullptr, toggleMappingPtr);
    EXPECT_EQ(mappingForCompare, *toggleMappingPtr);
}

TEST(CommandMapper, addPressMapping_successfully_adds_mapping_normal_case)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);
    RemoteMapState ms(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    PressCommandMapping mappingForCompare(&drivers, {&tc}, ms);

    cm.addMap(&mappingForCompare);
    const PressCommandMapping *pressMappingPtr =
        dynamic_cast<const PressCommandMapping *>(cm.getAtIndex(0));
    EXPECT_NE(nullptr, pressMappingPtr);
    EXPECT_EQ(mappingForCompare, *pressMappingPtr);
}
