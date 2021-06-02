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
#include <aruwlib/control/RemoteMapState.hpp>
#include <aruwlib/control/ToggleCommandMapping.hpp>
#include <gtest/gtest.h>

#include "TestCommand.hpp"
#include "TestSubsystem.hpp"

using namespace aruwlib::control;
using aruwlib::Drivers;
using aruwlib::Remote;

// Adding command with switch state RemoteMapState (RMS)

TEST(
    ToggleCommandMapping,
    executeCommandMapping_single_command_not_added_if_switch_based_RMS_is_not_subset)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2;
    ToggleCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand).Times(0);

    commandMapping.executeCommandMapping(ms2);
}

TEST(ToggleCommandMapping, executeCommandMapping_single_command_added_if_switch_based_RMS_is_equal)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2 = ms1;
    ToggleCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);

    commandMapping.executeCommandMapping(ms2);
}

TEST(ToggleCommandMapping, executeCommandMapping_single_command_added_if_switch_based_RMS_is_subset)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2 = ms1;
    ms2.initKeys(42);
    ms2.initRSwitch(Remote::SwitchState::UP);
    ms2.initLMouseButton();
    ToggleCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);

    commandMapping.executeCommandMapping(ms2);
}

// Adding commands, key based (including neg keys) RMS

TEST(
    ToggleCommandMapping,
    executeCommandMapping_single_command_not_added_if_key_based_RMS_is_not_subset)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B});
    RemoteMapState ms2;
    ToggleCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand).Times(0);

    commandMapping.executeCommandMapping(ms2);
}

TEST(ToggleCommandMapping, executeCommandMapping_single_command_added_if_key_based_RMS_is_equal)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B});
    RemoteMapState ms2 = ms1;
    ToggleCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);

    commandMapping.executeCommandMapping(ms2);
}

TEST(ToggleCommandMapping, executeCommandMapping_single_command_added_if_key_based_RMS_is_subset)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B});
    RemoteMapState ms2 = ms1;
    ms2.initLMouseButton();
    ms2.initLSwitch(Remote::SwitchState::DOWN);
    ToggleCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);

    commandMapping.executeCommandMapping(ms2);
}

TEST(
    ToggleCommandMapping,
    executeCommandMapping_single_command_not_added_if_key_based_RMS_with_neg_keys_contains_matching_neg_keys)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B}, {Remote::Key::C, Remote::Key::D});
    RemoteMapState ms2({Remote::Key::A, Remote::Key::B, Remote::Key::C, Remote::Key::D});
    ms2.initLMouseButton();
    ms2.initLSwitch(Remote::SwitchState::DOWN);
    ToggleCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand).Times(0);

    commandMapping.executeCommandMapping(ms2);
}

// Second stage of toggle command, match RMS to add command, then don't match RMS and match again,
// which should remove the command

TEST(ToggleCommandMapping, executeCommandMapping_single_command_removed_if_switch_based_RMS_toggled)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2 = ms1;
    ToggleCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);
    EXPECT_CALL(drivers.commandScheduler, removeCommand(&tc, false)).Times(1);

    commandMapping.executeCommandMapping(ms2);
    ms2 = RemoteMapState();
    commandMapping.executeCommandMapping(ms2);
    ms2 = ms1;
    commandMapping.executeCommandMapping(ms2);
}

TEST(
    ToggleCommandMapping,
    executeCommandMapping_single_command_not_removed_if_switch_based_RMS_never_equal_again)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2 = ms1;
    ms2.initKeys(42);
    ms2.initLMouseButton();
    ToggleCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);
    EXPECT_CALL(drivers.commandScheduler, removeCommand).Times(0);

    commandMapping.executeCommandMapping(ms2);
    ms2 = RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    commandMapping.executeCommandMapping(ms2);
    commandMapping.executeCommandMapping(ms2);
}

TEST(ToggleCommandMapping, executeCommandMapping_single_command_removed_if_key_based_RMS_toggled)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B}, {});
    RemoteMapState ms2({Remote::Key::A, Remote::Key::B, Remote::Key::C}, {});
    ToggleCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);
    EXPECT_CALL(drivers.commandScheduler, removeCommand).Times(1);

    commandMapping.executeCommandMapping(ms2);
    ms2 = RemoteMapState();
    commandMapping.executeCommandMapping(ms2);
    ms2 = ms1;
    commandMapping.executeCommandMapping(ms2);
}

TEST(
    ToggleCommandMapping,
    executeCommandMapping_single_command_removed_if_keys_match_then_neg_keys_match)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B}, {Remote::Key::C, Remote::Key::D});
    RemoteMapState ms2({Remote::Key::A, Remote::Key::B}, {});
    ToggleCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(2);
    EXPECT_CALL(drivers.commandScheduler, removeCommand(&tc, false)).Times(2);

    commandMapping.executeCommandMapping(ms2);
    ms2 = RemoteMapState(
        {Remote::Key::A, Remote::Key::B, Remote::Key::C, Remote::Key::D, Remote::Key::E});
    commandMapping.executeCommandMapping(ms2);  // command is now removed
    ms2 = RemoteMapState({Remote::Key::A, Remote::Key::B});
    commandMapping.executeCommandMapping(ms2);  // command is now added
    ms2 = RemoteMapState();
    commandMapping.executeCommandMapping(ms2);
    ms2 = RemoteMapState(
        {Remote::Key::A, Remote::Key::B, Remote::Key::C, Remote::Key::D, Remote::Key::E});
    commandMapping.executeCommandMapping(ms2);  // command is now removed
}

TEST(
    ToggleCommandMapping,
    executeCommandMapping_after_neg_key_removes_command_toggle_works_as_expected)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B}, {Remote::Key::C, Remote::Key::D});
    RemoteMapState ms2({Remote::Key::A, Remote::Key::B}, {});
    ToggleCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(2);
    EXPECT_CALL(drivers.commandScheduler, removeCommand(&tc, false)).Times(2);

    commandMapping.executeCommandMapping(ms2);
    ms2 = RemoteMapState(
        {Remote::Key::A, Remote::Key::B, Remote::Key::C, Remote::Key::D, Remote::Key::E});
    commandMapping.executeCommandMapping(ms2);  // command is now removed
    ms2 = RemoteMapState({Remote::Key::A, Remote::Key::B, Remote::Key::E});
    commandMapping.executeCommandMapping(ms2);  // command is now added
    ms2 = RemoteMapState();
    commandMapping.executeCommandMapping(ms2);
    ms2 = RemoteMapState({Remote::Key::A, Remote::Key::B, Remote::Key::C});
    commandMapping.executeCommandMapping(ms2);  // command is now removed
}
