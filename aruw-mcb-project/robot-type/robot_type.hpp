/**
 * @file robot_type.hpp
 *
 * @brief Header for intellisense
 *
 * This file exists solely to help syntax highlighting. We setup the VSCode configuration to
 * "include" this file for intellisense.
 *
 * `aruw-mcb` builds take a target option. This specifies which robot to build
 * for as different robots will have different subsystems and constants, so this file
 * allows the developer to view syntax highlighting for the robot they are developing
 * for to catch mistakes without having to rebuild the code every time.
 *
 * This file should **NOT** be included normally, as otherwise it will mess with the build system.
 */

#ifndef ROBOT_TYPE_HPP_
#define ROBOT_TYPE_HPP_

// Check that target isn't already defined. This would be caused by including the file
// when building (assuming define has been passed in through command line)
#if defined(TARGET_STANDARD_ELSA) || defined(TARGET_STANDARD_WOODY) || defined(TARGET_DRONE) ||   \
    defined(TARGET_ENGINEER) || defined(TARGET_SENTRY_BEEHIVE) || defined(TARGET_HERO_CYCLONE) || \
    defined(TARGET_STANDARD_SPIDER) || defined(TARGET_DART) || (TARGET_TESTBED)
#error "DON'T INCLUDE 'robot_type.hpp'!"
#endif

#include "robot_type_intellisense.hpp"

#endif
