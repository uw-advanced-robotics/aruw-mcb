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

// static void setupPressMapping(Subsystem *sub, Command *cmd, const RemoteMapState &mapState)
// {
//     Drivers::commandScheduler.registerSubsystem(sub);
//     Drivers::commandMapper.addPressMapping(mapState, {cmd});
// }

// TEST_CASE("PressCommandMapping: Simple test with switch", "[PressCommandMapping]")
// {
//     Drivers::reset();

//     // Create a subsystem and command and add the subsystem to the scheduler.
//     TestSubsystem ts;
//     TestCommand tc(&ts);

//     // Create and add a hold mapping.
//     RemoteMapState ms;
//     ms.initLSwitch(Remote::SwitchState::DOWN);

//     setupPressMapping(&ts, &tc, ms);

//     // Create a RemoteInfo struct that we will use to update the state of the remote manually.
//     Remote::RemoteInfo ri;

//     SECTION("Case 1. We activate the switch, the command is added")
//     {
//         ri.leftSwitch = Remote::SwitchState::DOWN;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         REQUIRE(tc.getTimesEnded() == 0);
//     }

//     SECTION(
//         "Case 2. We activate the switch and don't deactivate it. The command \
//             will eventually finish by itself and deactivating the switch won't \
//             change anything.")
//     {
//         ri.leftSwitch = Remote::SwitchState::DOWN;
//         Drivers::remote.read(ri);
//         Drivers::commandScheduler.run();
//         REQUIRE(tc.getTimesInited() == 1);
//         REQUIRE(tc.getTimesEnded() == 0);

//         Drivers::remote.read(ri);
//         Drivers::commandScheduler.run();
//         REQUIRE(tc.getTimesEnded() == 1);

//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//         REQUIRE(tc.getTimesEnded() == 1);
//     }
// }

// TEST_CASE("PressCommandMapping: Complex test with keys and neg keys", "[PressCommandMapping]")
// {
//     // We do this here so the mapping is reset after each section.
//     Drivers::reset();

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

//     setupPressMapping(&ts, &tc, ms);

//     // Create a RemoteInfo struct that we will use to update the state of the remote manually.
//     Remote::RemoteInfo ri;

//     ri.leftSwitch = Remote::SwitchState::DOWN;
//     ri.rightSwitch = Remote::SwitchState::UP;
//     ri.mouse.l = true;

//     SECTION("Case 1: the map state doesn't match (inclucing neg keys)")
//     {
//         ri.key = static_cast<uint16_t>(~(KEY_MAPPING | NEG_KEY_MAPPING));
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 0);
//     }

//     SECTION("Case 2: the map state matches exactly, no neg keys pressed")
//     {
//         ri.key = KEY_MAPPING;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//     }

//     SECTION("Case 3: the map state is a subset of the remote map state, no neg keys pressed")
//     {
//         ri.key = KEY_MAPPING | (0x1 << static_cast<uint16_t>(Remote::Key::E));
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//     }

//     SECTION("Case 4: the map state matches the remote map state, a single neg key is pressed")
//     {
//         ri.key = KEY_MAPPING | (0x1 << static_cast<uint16_t>(Remote::Key::C));
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 1);
//     }

//     SECTION("Case 5: the map state matches the remote map state, neg keys match as well")
//     {
//         ri.key = KEY_MAPPING | NEG_KEY_MAPPING;
//         Drivers::remote.read(ri);
//         REQUIRE(tc.getTimesInited() == 0);
//     }
// }
