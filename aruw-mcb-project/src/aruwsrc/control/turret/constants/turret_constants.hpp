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

#ifndef TURRET_CONSTANTS_HPP_
#define TURRET_CONSTANTS_HPP_

#include "aruwsrc/util_macros.hpp"

// TODO: remove all of these #error calls

#if defined(ALL_SOLDIERS) || defined(TARGET_ENGINEER)
#include "soldier_turret_constants.hpp"
#elif defined(TARGET_HERO)
#include "hero_turret_constants.hpp"
#elif defined(TARGET_DRONE)
#include "drone_turret_constants.hpp"
#elif defined(TARGET_SENTINEL_2021)
#include "sentinel_2021_turret_constants.hpp"
#elif defined(TARGET_DART)
#include "dart_turret_constants.hpp"
#elif defined(TARGET_SENTINEL_2022)
#include "sentinel_2022_turret_constants.hpp"
#else
#error Unexpected target
#endif

#endif  // TURRET_CONSTANTS_HPP_
