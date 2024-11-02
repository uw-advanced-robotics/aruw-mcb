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

#ifndef ENV_UNIT_TESTS

#include "drivers_singleton.hpp"

#if defined(ALL_STANDARDS)
namespace aruwsrc::standard
#elif defined(ALL_SENTRIES)
namespace aruwsrc::sentry
#elif defined(TARGET_HERO_PERSEUS)
namespace aruwsrc::hero
#elif defined(TARGET_DRONE)
namespace aruwsrc::drone
#elif defined(TARGET_ENGINEER)
namespace aruwsrc::engineer
#elif defined(TARGET_DART)
namespace aruwsrc::dart
#elif defined(TARGET_TESTBED)
namespace aruwsrc::testbed
#elif defined(TARGET_MOTOR_TESTER)
namespace aruwsrc::motor_tester
#endif
{
/**
 * Class that allows one to construct a Drivers instance because of friendship
 * with the Drivers class.
 */
class DriversSingleton
{
public:
    static Drivers drivers;
};  // class DriversSingleton

Drivers DriversSingleton::drivers;

Drivers *DoNotUse_getDrivers() { return &DriversSingleton::drivers; }
}  // namespace aruwsrc

#endif
