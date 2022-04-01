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

#ifndef CONSTANTS_HPP_
#define CONSTANTS_HPP_

#include "aruwsrc/util_macros.hpp"

#if defined(ALL_SOLDIERS)
#include "soldier_constants.hpp"
#elif defined(TARGET_HERO)
#include "hero_constants.hpp"
#elif defined(TARGET_SENTINEL)
#include "sentinel_constants.hpp"
#elif defined(TARGET_DRONE)
#include "drone_constants.hpp"
#elif defined(TARGET_ENGINEER)
#include "engineer_constants.hpp"
#endif

#endif  // CONSTANTS_HPP_
