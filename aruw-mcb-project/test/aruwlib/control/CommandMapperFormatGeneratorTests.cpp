// #include <iostream>
// #include <vector>

// #include <aruwlib/Drivers.hpp>
// #include <aruwlib/control/CommandMapperFormatGenerator.hpp>
// #include <catch/catch.hpp>

// #include "aruwsrc/control/robot_control.hpp"

// TEST_CASE("test formatter on robot control mappings")
// {
//     aruwsrc::control::initSubsystemCommands();

//     aruwlib::control::CommandMapperFormatGenerator
//     formatGenerator(aruwlib::Drivers::commandMapper);

//     std::vector<std::string> mappings = formatGenerator.generateMappings();

//     std::cout << "Generated map, check visually" << std::endl;
//     for (std::string map : mappings)
//     {
//         std::cout << map << std::endl;
//     }
// }
