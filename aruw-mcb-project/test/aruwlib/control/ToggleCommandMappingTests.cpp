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

// #include <aruwlib/Drivers.hpp>
// #include <aruwlib/control/CommandMapper.hpp>
// #include <aruwlib/control/CommandMapping.hpp>
// #include <aruwlib/control/RemoteMapState.hpp>

// #include "catch/catch.hpp"

// #include "TestCommand.hpp"
// #include "TestSubsystem.hpp"

// using namespace aruwlib::control;
// using aruwlib::Drivers;
// using aruwlib::Remote;

// static void setupToggleMapping(Subsystem *sub, Command *cmd, const RemoteMapState &mapState)
// {
//     Drivers::commandScheduler.registerSubsystem(sub);
//     Drivers::commandMapper.addToggleMapping(mapState, {cmd});
// }

// TEST_CASE("ToggleCommandMapping: Test with switch and mouse", "[ToggleCommandMapping]")
// {
//     Drivers::reset();

//     // Create a subsystem and command and add the subsystem to the scheduler.
//     TestSubsystem ts;
//     TestCommand tc(&ts);

//     // Create and add a hold mapping.
//     RemoteMapState ms;
//     ms.initLSwitch(Remote::SwitchState::DOWN);
//     ms.initLMouseButton();

//     setupToggleMapping(&ts, &tc, ms);

//     // Create a RemoteInfo struct that we will use to update the state of the remote manually.
//     Remote::RemoteInfo ri;

//     SECTION(
//         "Case 1: mapping matches exactly, initiating toggle, then mapping doesn't match, \
//         then mapping matches again, untoggling")
//     {
//         ri.leftSwitch = Remote::SwitchState::DOWN;
//         ri.mouse.l = true;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         ri.leftSwitch = Remote::SwitchState::UP;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         REQUIRE(tc.getTimesEnded() == 0);
//         ri.leftSwitch = Remote::SwitchState::DOWN;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         REQUIRE(tc.getTimesEnded() == 1);
//     }

//     SECTION("Case 2: same as case 1, with some extra remote changes in between")
//     {
//         ri.leftSwitch = Remote::SwitchState::DOWN;
//         ri.key = 0x3421;
//         ri.mouse.r = true;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 0);
//         ri.mouse.l = true;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         REQUIRE(tc.getTimesEnded() == 0);
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         REQUIRE(tc.getTimesEnded() == 0);
//         ri.leftSwitch = Remote::SwitchState::MID;
//         ri.rightSwitch = Remote::SwitchState::UP;
//         ri.key = 0x4239;
//         ri.mouse.r = false;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         REQUIRE(tc.getTimesEnded() == 0);
//         ri.rightSwitch = Remote::SwitchState::DOWN;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         REQUIRE(tc.getTimesEnded() == 0);
//         ri.leftSwitch = Remote::SwitchState::DOWN;
//         ri.mouse.l = true;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         REQUIRE(tc.getTimesEnded() == 1);
//     }
// }

// TEST_CASE("ToggleCommandMapping: Test with keys and neg keys", "[ToggleCommandMapping]")
// {
//     Drivers::reset();

//     // Create a subsystem and command and add the subsystem to the scheduler.
//     TestSubsystem ts;
//     TestCommand tc(&ts);

//     const uint16_t KEY_MAPPING = (0x1 << static_cast<uint16_t>(Remote::Key::A)) |
//                                  (0x1 << static_cast<uint16_t>(Remote::Key::B));
//     const uint16_t NEG_KEY_MAPPING = (0x1 << static_cast<uint16_t>(Remote::Key::C)) |
//                                      (0x1 << static_cast<uint16_t>(Remote::Key::D));

//     // Create and add a hold mapping.
//     RemoteMapState ms;
//     ms.initKeys(KEY_MAPPING);
//     ms.initNegKeys(NEG_KEY_MAPPING);

//     setupToggleMapping(&ts, &tc, ms);

//     // Create a RemoteInfo struct that we will use to update the state of the remote manually.
//     Remote::RemoteInfo ri;

//     SECTION("Case 1: Simple case, no neg keys")
//     {
//         ri.key = KEY_MAPPING;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         ri.key = 0;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         REQUIRE(tc.getTimesEnded() == 0);
//         ri.key = KEY_MAPPING;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         REQUIRE(tc.getTimesEnded() == 1);
//     }

//     SECTION("Case 2: some neg keys")
//     {
//         ri.key = KEY_MAPPING | (0x1 << static_cast<uint16_t>(Remote::Key::C));
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         ri.key = 0;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         REQUIRE(tc.getTimesEnded() == 0);
//         ri.key = KEY_MAPPING | (0x1 << static_cast<uint16_t>(Remote::Key::C));
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         REQUIRE(tc.getTimesEnded() == 1);
//     }

//     SECTION("Case 3: neg key matches while first toggling")
//     {
//         ri.key = KEY_MAPPING | NEG_KEY_MAPPING;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 0);
//         ri.key = KEY_MAPPING;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//     }

//     SECTION("Case 4: neg key matches after initially toggling")
//     {
//         ri.key = KEY_MAPPING;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         ri.key = KEY_MAPPING | NEG_KEY_MAPPING;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         REQUIRE(tc.getTimesEnded() == 1);
//         ri.key = KEY_MAPPING;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 2);
//         REQUIRE(tc.getTimesEnded() == 1);
//         ri.key = 0;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 2);
//         REQUIRE(tc.getTimesEnded() == 1);
//         ri.key = KEY_MAPPING;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 2);
//         REQUIRE(tc.getTimesEnded() == 2);
//     }

//     SECTION("Case 5: neg key matches during ungoggle phase")
//     {
//         ri.key = KEY_MAPPING;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         ri.key = 0;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         REQUIRE(tc.getTimesEnded() == 0);
//         ri.key = KEY_MAPPING | NEG_KEY_MAPPING;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         REQUIRE(tc.getTimesEnded() == 1);

//         ri.key = KEY_MAPPING;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 2);
//         REQUIRE(tc.getTimesEnded() == 1);
//         ri.key = 0;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 2);
//         REQUIRE(tc.getTimesEnded() == 1);
//         ri.key = KEY_MAPPING;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 2);
//         REQUIRE(tc.getTimesEnded() == 2);
//     }
// }
