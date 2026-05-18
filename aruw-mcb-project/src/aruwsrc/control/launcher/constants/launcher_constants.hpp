/*
 * Copyright (c) 2020-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef LAUNCHER_CONSTANTS_HPP_ 
#define LAUNCHER_CONSTANTS_HPP_

#include "aruwsrc/util_macros.hpp"

#if defined(ALL_STANDARDS)
#include "aruwsrc/robot/standard/standard_launcher_constants.hpp"
#elif defined(TARGET_HERO_NEPTUNE)
#include "aruwsrc/robot/hero/hero_launcher_constants.hpp"
#elif defined(TARGET_SENTRY_ACHLYS)
#include "aruwsrc/robot/sentry/sentry_launcher_constants.hpp"
#elif defined(TARGET_DRONE)
#include "aruwsrc/robot/drone/drone_launcher_constants.hpp"
#elif defined(TARGET_FLYWHEEL_TESTING)
#include "aruwsrc/robot/flywheel_testing/flywheel_testing_launcher_constants.hpp"
#else  // by default use standard constants (for robots that don't use them)
#include "aruwsrc/robot/standard/standard_launcher_constants.hpp"
#endif

#endif  // LAUNCHER_CONSTANTS_HPP_
