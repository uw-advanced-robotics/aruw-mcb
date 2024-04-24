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

#ifndef AGITATOR_CONSTANTS_HPP_
#define AGITATOR_CONSTANTS_HPP_

#if defined(ALL_STANDARDS)
#include "aruwsrc/robot/standard/standard_agitator_constants.hpp"
#elif defined(TARGET_SENTRY_BEEHIVE)
#include "aruwsrc/robot/sentry/sentry_beehive_agitator_constants.hpp"
#elif defined(TARGET_HERO_MEGATRON)
#include "aruwsrc/robot/hero/hero_agitator_constants.hpp"
#endif

#endif  // AGITATOR_CONSTANTS_HPP_
