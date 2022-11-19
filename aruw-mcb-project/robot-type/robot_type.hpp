/**
 * @file robot_type.hpp
 *
 * @brief Header for intellisense and default robot selection
 *
 * This file serves two main purposes:
 *  1. To provide a default build target if one isn't provided when building.
 *     This functionality is handled by the build script searching this file.
 *  2. To help syntax highlighting. We setup the VSCode configuration to
 *     "include" this file for intellisense
 *
 * `aruw-mcb` builds take a target option. This specifies which robot to build
 * for as different robots will have different subsystems and constants etc., and
 * this file helps with the above mentioned purposes.
 *
 * So as to not mess with the build system this file should **NOT** be included
 * normally.
 */

#ifndef ROBOT_TYPE_HPP_
#define ROBOT_TYPE_HPP_

// Check that target isn't already defined. This would be caused by including the file
// when building (assuming define has been passed in through command line)
#if defined(TARGET_STANDARD_ELSA) || defined(TARGET_STANDARD_WOODY) || defined(TARGET_DRONE) ||      \
    defined(TARGET_ENGINEER) || defined(TARGET_SENTRY_BEEHIVE) || \
    defined(TARGET_HERO_CYCLONE) || defined(TARGET_STANDARD_SPIDER)
#error "DON'T INCLUDE 'robot_type.hpp'!"
#endif

#define TARGET_STANDARD_SPIDER

#endif
