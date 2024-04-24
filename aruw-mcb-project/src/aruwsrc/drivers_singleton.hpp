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

#ifndef DRIVERS_SINGLETON_HPP_
#define DRIVERS_SINGLETON_HPP_

#ifndef ENV_UNIT_TESTS

#include "aruwsrc/util_macros.hpp"

#if defined(ALL_STANDARDS)
#include "aruwsrc/robot/standard/standard_drivers.hpp"
namespace aruwsrc::standard
#elif defined(ALL_SENTRIES)
#include "aruwsrc/robot/sentry/sentry_drivers.hpp"
namespace aruwsrc::sentry
#elif defined(TARGET_HERO_KRONOS)
#include "aruwsrc/robot/hero/hero_drivers.hpp"
namespace aruwsrc::hero
#elif defined(TARGET_DRONE)
#include "aruwsrc/robot/drone/drone_drivers.hpp"
namespace aruwsrc::drone
#elif defined(TARGET_ENGINEER)
#include "aruwsrc/robot/engineer/engineer_drivers.hpp"
namespace aruwsrc::engineer
#elif defined(TARGET_DART)
#include "aruwsrc/robot/dart/dart_drivers.hpp"
namespace aruwsrc::dart
#elif defined(TARGET_TESTBED)
#include "aruwsrc/robot/testbed/testbed_drivers.hpp"
namespace aruwsrc::testbed
#endif
{
/**
 * @return The singleton instance of the Drivers class. This is the only instance of the
 *      Drivers class that should be created anywhere in the non-unit test framework.
 * @note It is likely that you will never have to use this. There are only two files you
 *      should be calling this function from -- `main.cpp` and `*_control.cpp`, either to
 *      run I/O stuff and to add a Drivers pointer to an instance of a Subsystem or Command.
 */
Drivers *DoNotUse_getDrivers();
using driversFunc = Drivers *(*)();
}  // namespace aruwsrc

#endif

#endif  // DRIVERS_SINGLETON_HPP_
