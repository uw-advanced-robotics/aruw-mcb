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

// static void setupHoldMapping(Subsystem *sub, Command *cmd, const RemoteMapState &mapState)
// {
//     Drivers::commandScheduler.registerSubsystem(sub);
//     Drivers::commandMapper.addHoldMapping(mapState, {cmd});
// }

// TEST_CASE("HoldCommandMapping: Simple test with switch", "[HoldCommandMapping]")
// {
//     // Create a subsystem and command and add the subsystem to the scheduler.
//     TestSubsystem ts;
//     TestCommand tc(&ts);

//     // Create and add a hold mapping.
//     RemoteMapState ms;
//     ms.initLSwitch(Remote::SwitchState::DOWN);

//     setupHoldMapping(&ts, &tc, ms);

//     // Create a RemoteInfo struct that we will use to update the state of the remote manually.
//     Remote::RemoteInfo ri;
//     ri.leftSwitch = Remote::SwitchState::DOWN;

//     // The command should be added when reading.
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 1);
//     REQUIRE(tc.getTimesEnded() == 0);

//     // Switching the remote will remove the command.
//     ri.leftSwitch = Remote::SwitchState::MID;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 1);
//     REQUIRE(tc.getTimesEnded() == 1);

//     // Switching the remote state again will toggle the command.
//     ri.leftSwitch = Remote::SwitchState::DOWN;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 2);
//     REQUIRE(tc.getTimesEnded() == 1);

//     // And finally switching back will untoggle again.
//     ri.leftSwitch = Remote::SwitchState::MID;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 2);
//     REQUIRE(tc.getTimesEnded() == 2);
// }

// TEST_CASE("HoldCommandMapping: Complex test with switch", "[HoldCommandMapping]")
// {
//     // Create a subsystem and command and add the subsystem to the scheduler.
//     TestSubsystem ts;
//     TestCommand tc(&ts);

//     // Create and add a hold mapping.
//     RemoteMapState ms;
//     ms.initLSwitch(Remote::SwitchState::DOWN);

//     setupHoldMapping(&ts, &tc, ms);

//     // Create a RemoteInfo struct that we will use to update the state of the remote manually.
//     Remote::RemoteInfo ri;

//     // Switch random switch minus moving the left switch down. The command should not be
//     // added.
//     ri.leftSwitch = Remote::SwitchState::MID;
//     ri.rightSwitch = Remote::SwitchState::DOWN;
//     ri.key = 0b1010101010101010;
//     ri.mouse.l = true;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 0);
//     REQUIRE(tc.getTimesEnded() == 0);

//     // Change the state of the remote some more to insure the command is not toggled.
//     ri.leftSwitch = Remote::SwitchState::UP;
//     ri.rightSwitch = Remote::SwitchState::UNKNOWN;
//     ri.key = 0b1100010011101100;
//     ri.mouse.l = false;
//     ri.mouse.r = true;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 0);
//     REQUIRE(tc.getTimesEnded() == 0);

//     // Now initiate the toggle state.
//     ri.leftSwitch = Remote::SwitchState::DOWN;
//     ri.mouse.l = true;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 1);
//     REQUIRE(tc.getTimesEnded() == 0);

//     // Change the state some more.
//     ri.rightSwitch = Remote::SwitchState::DOWN;
//     ri.key = 0b1011100110011011;
//     ri.mouse.r = false;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 1);
//     REQUIRE(tc.getTimesEnded() == 0);

//     ri.rightSwitch = Remote::SwitchState::MID;
//     ri.key = 0;
//     ri.mouse.r = true;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 1);
//     REQUIRE(tc.getTimesEnded() == 0);

//     // Untoggle.
//     ri.leftSwitch = Remote::SwitchState::MID;
//     ri.rightSwitch = Remote::SwitchState::DOWN;
//     ri.key = 0b0010001100101000;
//     ri.mouse.l = false;
//     ri.mouse.r = true;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 1);
//     REQUIRE(tc.getTimesEnded() == 1);
// }

// TEST_CASE("HoldCommandMapping: Simple test with keys", "[HoldCommandMapping]")
// {
//     // Create a subsystem and command and add the subsystem to the scheduler.
//     TestSubsystem ts;
//     TestCommand tc(&ts);

//     // Create and add a hold mapping.
//     const uint16_t MAP_KEY = 0X1234;
//     RemoteMapState ms;
//     ms.initKeys(MAP_KEY);

//     setupHoldMapping(&ts, &tc, ms);

//     // Create a RemoteInfo struct that we will use to update the state of the remote manually.
//     Remote::RemoteInfo ri;
//     ri.key = MAP_KEY;

//     // The command should be added when reading.
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 1);
//     REQUIRE(tc.getTimesEnded() == 0);

//     // Switching the remote will remove the command.
//     ri.key = MAP_KEY & 4;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 1);
//     REQUIRE(tc.getTimesEnded() == 1);

//     // Switching the remote state again will toggle the command.
//     ri.key = MAP_KEY;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 2);
//     REQUIRE(tc.getTimesEnded() == 1);

//     // And finally switching back will untoggle again.
//     ri.key = MAP_KEY - 1;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 2);
//     REQUIRE(tc.getTimesEnded() == 2);
// }

// TEST_CASE("HoldCommandMapping: Complex test with keys", "[HoldCommandMapping]")
// {
//     // Create a subsystem and command and add the subsystem to the scheduler.
//     TestSubsystem ts;
//     TestCommand tc(&ts);

//     // Create and add a hold mapping.
//     const uint16_t MAP_KEY = 0X1234;
//     RemoteMapState ms;
//     ms.initKeys(MAP_KEY);

//     setupHoldMapping(&ts, &tc, ms);

//     // Create a RemoteInfo struct that we will use to update the state of the remote manually.
//     Remote::RemoteInfo ri;

//     // Switch random switch minus moving the left switch down. The command should not be
//     // added.
//     ri.leftSwitch = Remote::SwitchState::MID;
//     ri.rightSwitch = Remote::SwitchState::DOWN;
//     ri.key = MAP_KEY & 4;
//     ri.mouse.l = true;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 0);
//     REQUIRE(tc.getTimesEnded() == 0);

//     // Change the state of the remote some more to insure the command is not toggled.
//     ri.leftSwitch = Remote::SwitchState::UP;
//     ri.rightSwitch = Remote::SwitchState::UNKNOWN;
//     ri.key = MAP_KEY & 0x30;
//     ri.mouse.l = false;
//     ri.mouse.r = true;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 0);
//     REQUIRE(tc.getTimesEnded() == 0);

//     // Now initiate the toggle state.
//     ri.leftSwitch = Remote::SwitchState::DOWN;
//     ri.rightSwitch = Remote::SwitchState::MID;
//     ri.key = MAP_KEY;
//     ri.mouse.l = true;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 1);
//     REQUIRE(tc.getTimesEnded() == 0);

//     // Change the state some more.
//     ri.rightSwitch = Remote::SwitchState::DOWN;
//     ri.key = MAP_KEY;
//     ri.mouse.r = false;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 1);
//     REQUIRE(tc.getTimesEnded() == 0);

//     ri.leftSwitch = Remote::SwitchState::UP;
//     ri.rightSwitch = Remote::SwitchState::MID;
//     ri.key = MAP_KEY;
//     ri.mouse.r = true;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 1);
//     REQUIRE(tc.getTimesEnded() == 0);

//     // Untoggle.
//     ri.leftSwitch = Remote::SwitchState::MID;
//     ri.rightSwitch = Remote::SwitchState::DOWN;
//     ri.key = 0;
//     ri.mouse.l = false;
//     ri.mouse.r = true;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 1);
//     REQUIRE(tc.getTimesEnded() == 1);
// }

// TEST_CASE("HoldCommandMapping: Complex test with keys and neg keys", "[HoldCommandMapping]")
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

//     setupHoldMapping(&ts, &tc, ms);

//     // Create a RemoteInfo struct that we will use to update the state of the remote manually.
//     Remote::RemoteInfo ri;

//     // Simple case: the map state for the keys match exactly.
//     ri.key = KEY_MAPPING;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 1);
//     REQUIRE(tc.getTimesEnded() == 0);

//     // Remove the command from the scheduler.
//     ri.key = 0;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 1);
//     REQUIRE(tc.getTimesEnded() == 1);

//     // Case 2: the map state for the keys match and one of the neg key matches,
//     // still added to the scheduler.
//     ri.key = KEY_MAPPING | (0x1 << static_cast<uint16_t>(Remote::Key::A));
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 2);
//     REQUIRE(tc.getTimesEnded() == 1);

//     // Remove the command from the scheduler.
//     ri.key = 0;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 2);
//     REQUIRE(tc.getTimesEnded() == 2);

//     // Case 3: the map state for keys and neg keys matches, don't add to scheduler.
//     ri.key = KEY_MAPPING | NEG_KEY_MAPPING;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 2);
//     REQUIRE(tc.getTimesInited() == 2);

//     // Case 4: the map state matches initially, so the command is added, but then
//     // the map state changes such that the neg key matches and the command is removed
//     ri.key = KEY_MAPPING;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 3);
//     REQUIRE(tc.getTimesEnded() == 2);
//     ri.key = KEY_MAPPING | (0x1 << static_cast<uint16_t>(Remote::Key::A));
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesEnded() == 2);
//     ri.key = KEY_MAPPING | NEG_KEY_MAPPING;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesEnded() == 3);
// }

// TEST_CASE("HoldCommandMapping: Simple test with mouse", "[HoldCommandMapping]")
// {
//     // Create a subsystem and command and add the subsystem to the scheduler.
//     TestSubsystem ts;
//     TestCommand tc(&ts);

//     // Create and add a hold mapping.
//     RemoteMapState ms;
//     ms.initRMouseButton();

//     setupHoldMapping(&ts, &tc, ms);

//     // Create a RemoteInfo struct that we will use to update the state of the remote manually.
//     Remote::RemoteInfo ri;
//     ri.mouse.r = true;

//     // The command should be added when reading.
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 1);
//     REQUIRE(tc.getTimesEnded() == 0);

//     // Switching the remote will remove the command.
//     ri.mouse.r = false;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 1);
//     REQUIRE(tc.getTimesEnded() == 1);

//     // Switching the remote state again will toggle the command.
//     ri.mouse.r = true;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 2);
//     REQUIRE(tc.getTimesEnded() == 1);

//     // And finally switching back will untoggle again.
//     ri.mouse.r = false;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 2);
//     REQUIRE(tc.getTimesEnded() == 2);
// }

// TEST_CASE("HoldCommandMapping: Complex test with mouse", "[HoldCommandMapping]")
// {
//     // Create a subsystem and command and add the subsystem to the scheduler.
//     TestSubsystem ts;
//     TestCommand tc(&ts);

//     // Create and add a hold mapping.
//     RemoteMapState ms;
//     ms.initRMouseButton();

//     setupHoldMapping(&ts, &tc, ms);

//     // Create a RemoteInfo struct that we will use to update the state of the remote manually.
//     Remote::RemoteInfo ri;

//     // Switch random switch minus moving the left switch down. The command should not be
//     // added.
//     ri.leftSwitch = Remote::SwitchState::MID;
//     ri.rightSwitch = Remote::SwitchState::DOWN;
//     ri.key = 0x4231;
//     ri.mouse.l = true;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 0);
//     REQUIRE(tc.getTimesEnded() == 0);

//     // Change the state of the remote some more to insure the command is not toggled.
//     ri.leftSwitch = Remote::SwitchState::UP;
//     ri.rightSwitch = Remote::SwitchState::UNKNOWN;
//     ri.key = 0x1446;
//     ri.mouse.l = false;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 0);
//     REQUIRE(tc.getTimesEnded() == 0);

//     // Now initiate the toggle state.
//     ri.leftSwitch = Remote::SwitchState::DOWN;
//     ri.rightSwitch = Remote::SwitchState::MID;
//     ri.key = 0x1250;
//     ri.mouse.l = true;
//     ri.mouse.r = true;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 1);
//     REQUIRE(tc.getTimesEnded() == 0);

//     // Change the state some more.
//     ri.rightSwitch = Remote::SwitchState::DOWN;
//     ri.key = 0x0876;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 1);
//     REQUIRE(tc.getTimesEnded() == 0);

//     ri.leftSwitch = Remote::SwitchState::UP;
//     ri.rightSwitch = Remote::SwitchState::MID;
//     ri.key = 0x5239;
//     ri.mouse.l = false;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 1);
//     REQUIRE(tc.getTimesEnded() == 0);

//     // Untoggle.
//     ri.leftSwitch = Remote::SwitchState::MID;
//     ri.rightSwitch = Remote::SwitchState::DOWN;
//     ri.key = 0;
//     ri.mouse.r = false;
//     Drivers::remote.read(ri);
//     REQUIRE(tc.getTimesInited() == 1);
//     REQUIRE(tc.getTimesEnded() == 1);
// }
