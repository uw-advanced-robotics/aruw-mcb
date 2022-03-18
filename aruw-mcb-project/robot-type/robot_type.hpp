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
 * normally unless you're sure of what you're doing.
 */

#ifndef __ROBOT_TYPE_HPP__
#define __ROBOT_TYPE_HPP__
#define TARGET_SOLDIER_2022
#endif
