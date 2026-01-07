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

#ifndef ROBOT_CONTROL_HPP_
#define ROBOT_CONTROL_HPP_

#if defined(ALL_STANDARDS)
#include "aruwsrc/robot/standard/standard_drivers.hpp"
namespace aruwsrc::standard
#elif defined(ALL_SENTRIES)
#include "aruwsrc/robot/sentry/sentry_drivers.hpp"
namespace aruwsrc::sentry
#elif defined(TARGET_HERO_ZERO)
#include "aruwsrc/robot/hero/hero_drivers.hpp"
namespace aruwsrc::hero
#elif defined(TARGET_DRONE)
#include "aruwsrc/robot/drone/drone_drivers.hpp"
namespace aruwsrc::drone
#elif defined(TARGET_ENGINEER)
#include "aruwsrc/robot/engineer/engineer_drivers.hpp"
#elif defined(TARGET_2025_ENGINEER)
#include "aruwsrc/robot/engineer/engineer_drivers.hpp"
namespace aruwsrc::engineer
#elif defined(TARGET_DART)
namespace aruwsrc::dart
#elif defined(TARGET_TESTBED)
#include "aruwsrc/robot/testbed/testbed_drivers.hpp"
namespace aruwsrc::testbed
#elif defined(TARGET_BLANK)
#include "aruwsrc/robot/blank/blank_drivers.hpp"
namespace aruwsrc::blank
#elif defined(TARGET_MOTOR_TESTER)
#include "aruwsrc/robot/motor_tester/motor_tester_drivers.hpp"
namespace aruwsrc::motor_tester
#elif defined(TARGET_CHARACTERIZER)
#include "aruwsrc/robot/characterizer/characterizer_drivers.hpp"
namespace aruwsrc::characterizer
#endif
{
void initSubsystemCommands(Drivers *drivers);
}  // namespace tbh whatever you want it to be

#endif  // ROBOT_CONTROL_HPP_
