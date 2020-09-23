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

// #include <iostream>

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

// static void setupHoldRepeatMapping(Subsystem *sub, Command *cmd, const RemoteMapState &mapState)
// {
//     Drivers::commandScheduler.registerSubsystem(sub);
//     Drivers::commandMapper.addHoldRepeatMapping(mapState, {cmd});
// }

// TEST_CASE("HoldRepeatCommandMapping: Simple test with switch", "[HoldRepeatCommandMapping]")
// {
//     Drivers::reset();

//     // Create a subsystem and command and add the subsystem to the scheduler.
//     TestSubsystem ts;
//     TestCommand tc(&ts);

//     // Create and add a hold mapping.
//     RemoteMapState ms;
//     ms.initLSwitch(Remote::SwitchState::DOWN);

//     setupHoldRepeatMapping(&ts, &tc, ms);

//     // Create a RemoteInfo struct that we will use to update the state of the remote manually.
//     Remote::RemoteInfo ri;
//     ri.leftSwitch = Remote::SwitchState::DOWN;

//     // The command should be added when reading.
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 1);
//     REQUIRE(tc.getTimesEnded() == 0);

//     for (int i = 0; i < TestCommand::EXECUTE_COUNTS_BEFORE_ENDING; i++)
//     {
//         REQUIRE(tc.getTimesInited() == 1);
//         REQUIRE(tc.getTimesEnded() == 0);
//         Drivers::remote.read(ri);
//         Drivers::commandScheduler.run();
//     }

//     // Since the mapping still matches, the command will be removed when the command scheduler
//     // is ran next and added again when the remote is ran.
//     REQUIRE(tc.getTimesEnded() == 1);
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 2);
//     REQUIRE(tc.getTimesEnded() == 1);

//     // When the mapping doesn't match, the command is removed again.
//     ri.leftSwitch = Remote::SwitchState::MID;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesEnded() == 2);
// }

// TEST_CASE("HoldRepeatCommandMapping: Simple test with keys", "[HoldRepeatCommandMapping]")
// {
//     // Create a subsystem and command and add the subsystem to the scheduler.
//     TestSubsystem ts;
//     TestCommand tc(&ts);

//     // Create and add a hold mapping.
//     const uint16_t MAP_KEY = 0X1234;
//     RemoteMapState ms;
//     ms.initKeys(MAP_KEY);

//     setupHoldRepeatMapping(&ts, &tc, ms);

//     // Create a RemoteInfo struct that we will use to update the state of the remote manually.
//     Remote::RemoteInfo ri;
//     ri.key = MAP_KEY;

//     // The command should be added when reading.
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 1);
//     REQUIRE(tc.getTimesEnded() == 0);

//     for (int i = 0; i < TestCommand::EXECUTE_COUNTS_BEFORE_ENDING; i++)
//     {
//         REQUIRE(tc.getTimesInited() == 1);
//         REQUIRE(tc.getTimesEnded() == 0);
//         Drivers::remote.read(ri);
//         Drivers::commandScheduler.run();
//     }

//     // The command should now be removed
//     REQUIRE(tc.getTimesEnded() == 1);

//     // The command should now be added again.
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 2);

//     // Switching the remote will remove the command.
//     ri.key = MAP_KEY & 4;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 2);
//     REQUIRE(tc.getTimesEnded() == 2);
// }

// TEST_CASE(
//     "HoldRepeatCommandMapping: Complex test with keys and neg keys",
//     "[HoldRepeatCommandMapping]")
// {
//     // Create a subsystem and command and add the subsystem to the scheduler.
//     TestSubsystem ts;
//     TestCommand tc(&ts);

//     // Create and add a hold mapping with some keys and neg keys.
//     const uint16_t KEY_MAPPING = (0x1 << static_cast<uint16_t>(Remote::Key::A)) |
//                                  (0x1 << static_cast<uint16_t>(Remote::Key::B));
//     const uint16_t NEG_KEY_MAPPING = (0x1 << static_cast<uint16_t>(Remote::Key::C)) |
//                                      (0x1 << static_cast<uint16_t>(Remote::Key::D));
//     RemoteMapState ms;
//     ms.initKeys(KEY_MAPPING);
//     ms.initNegKeys(NEG_KEY_MAPPING);

//     setupHoldRepeatMapping(&ts, &tc, ms);

//     // Create a RemoteInfo struct that we will use to update the state of the remote manually.
//     Remote::RemoteInfo ri;

//     SECTION("Simple case: the map state for the keys match exactly")
//     {
//         ri.key = KEY_MAPPING;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         REQUIRE(tc.getTimesEnded() == 0);

//         // Let the command expire, insure the mapper adds it again
//         for (int i = 0; i < TestCommand::EXECUTE_COUNTS_BEFORE_ENDING; i++)
//         {
//             REQUIRE(tc.getTimesInited() == 1);
//             REQUIRE(tc.getTimesEnded() == 0);
//             Drivers::remote.read(ri);
//             Drivers::commandScheduler.run();
//         }

//         REQUIRE(tc.getTimesEnded() == 1);
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 2);
//         REQUIRE(tc.getTimesEnded() == 1);

//         // Remove the command from the scheduler.
//         ri.key = 0;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 2);
//         REQUIRE(tc.getTimesEnded() == 2);
//     }

//     SECTION(
//         "Case 2: the map state for the keys match and one of the neg key matches,
//         still added to the scheduler.")
//     {
//         ri.key = KEY_MAPPING | (0x1 << static_cast<uint16_t>(Remote::Key::A));
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         REQUIRE(tc.getTimesEnded() == 0);

//         // Let the command expire, insure the mapper adds it again
//         for (int i = 0; i < TestCommand::EXECUTE_COUNTS_BEFORE_ENDING; i++)
//         {
//             REQUIRE(tc.getTimesInited() == 1);
//             REQUIRE(tc.getTimesEnded() == 0);
//             Drivers::remote.read(ri);
//             Drivers::commandScheduler.run();
//         }

//         REQUIRE(tc.getTimesEnded() == 1);
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 2);
//         REQUIRE(tc.getTimesEnded() == 1);

//         // Remove the command from the scheduler.
//         ri.key = 0;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 2);
//         REQUIRE(tc.getTimesEnded() == 2);
//     }

//     SECTION("Case 3: the map state for keys and neg keys matches, don't add to scheduler")
//     {
//         ri.key = KEY_MAPPING | NEG_KEY_MAPPING;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 0);
//     }

//     SECTION(
//         "Case 4: the map state matches initially, so the command is added, but then
//         the map state changes such that the neg key matches and the command is removed")
//     {
//         ri.key = KEY_MAPPING;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         REQUIRE(tc.getTimesEnded() == 0);
//         ri.key = KEY_MAPPING | (0x1 << static_cast<uint16_t>(Remote::Key::A));
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesEnded() == 0);
//         ri.key = KEY_MAPPING | NEG_KEY_MAPPING;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesEnded() == 1);
//     }

//     SECTION(
//         "Case 5: the map state matches initially, so the command is added,
//         and the command expires, but when it should be added again, the neg key
//         matches so the command isn't added")
//     {
//         ri.key = KEY_MAPPING;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         REQUIRE(tc.getTimesEnded() == 0);

//         // Let the command expire, insure the mapper adds it again
//         for (int i = 0; i < TestCommand::EXECUTE_COUNTS_BEFORE_ENDING; i++)
//         {
//             REQUIRE(tc.getTimesInited() == 1);
//             REQUIRE(tc.getTimesEnded() == 0);
//             Drivers::remote.read(ri);
//             Drivers::commandScheduler.run();
//         }

//         REQUIRE(tc.getTimesEnded() == 1);
//         ri.key = KEY_MAPPING | NEG_KEY_MAPPING;
//         // The command shouldn't be added
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         REQUIRE(tc.getTimesEnded() == 1);
//     }
// }

// TEST_CASE("HoldRepeatCommandMapping: Complex test with mouse", "[HoldRepeatCommandMapping]")
// {
//     // Create a subsystem and command and add the subsystem to the scheduler.
//     TestSubsystem ts;
//     TestCommand tc(&ts);

//     // Create and add a hold mapping.
//     RemoteMapState ms;
//     ms.initRMouseButton();

//     setupHoldRepeatMapping(&ts, &tc, ms);

//     // Create a RemoteInfo struct that we will use to update the state of the remote manually.
//     Remote::RemoteInfo ri;

//     SECTION(
//         "Switch random switch minus moving the left switch down. The
//         command should not be added")
//     {
//         ri.leftSwitch = Remote::SwitchState::MID;
//         ri.rightSwitch = Remote::SwitchState::DOWN;
//         ri.key = 0x4231;
//         ri.mouse.l = true;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 0);
//         REQUIRE(tc.getTimesEnded() == 0);
//     }

//     SECTION("Change the state of the remote some more to insure the command is not toggled")
//     {
//         ri.leftSwitch = Remote::SwitchState::UP;
//         ri.rightSwitch = Remote::SwitchState::UNKNOWN;
//         ri.key = 0x1446;
//         ri.mouse.l = false;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 0);
//         REQUIRE(tc.getTimesEnded() == 0);
//     }

//     SECTION("Random state change")
//     {
//         // Now initiate the toggle state.
//         ri.leftSwitch = Remote::SwitchState::DOWN;
//         ri.rightSwitch = Remote::SwitchState::MID;
//         ri.key = 0x1250;
//         ri.mouse.l = true;
//         ri.mouse.r = true;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         REQUIRE(tc.getTimesEnded() == 0);

//         // Change the state some more, don't end it.
//         ri.rightSwitch = Remote::SwitchState::DOWN;
//         ri.key = 0x0876;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         REQUIRE(tc.getTimesEnded() == 0);

//         // Let the command expire, insure the mapper adds it again
//         for (int i = 0; i < TestCommand::EXECUTE_COUNTS_BEFORE_ENDING; i++)
//         {
//             REQUIRE(tc.getTimesInited() == 1);
//             REQUIRE(tc.getTimesEnded() == 0);
//             Drivers::remote.read(ri);
//             Drivers::commandScheduler.run();
//         }

//         ri.leftSwitch = Remote::SwitchState::UP;
//         ri.rightSwitch = Remote::SwitchState::MID;
//         ri.key = 0x5239;
//         ri.mouse.l = false;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 2);
//         REQUIRE(tc.getTimesEnded() == 1);

//         // Let the command expire, insure the mapper adds it again
//         for (int i = 0; i < TestCommand::EXECUTE_COUNTS_BEFORE_ENDING; i++)
//         {
//             REQUIRE(tc.getTimesInited() == 2);
//             REQUIRE(tc.getTimesEnded() == 1);
//             Drivers::remote.read(ri);
//             Drivers::commandScheduler.run();
//         }

//         // Untoggle.
//         ri.leftSwitch = Remote::SwitchState::MID;
//         ri.rightSwitch = Remote::SwitchState::DOWN;
//         ri.key = 0;
//         ri.mouse.r = false;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 2);
//         REQUIRE(tc.getTimesEnded() == 2);
//     }
// }
