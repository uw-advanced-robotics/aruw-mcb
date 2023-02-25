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

#include "aruwsrc/robot/drone/drone_drivers.hpp"
#include "aruwsrc/robot/engineer/engineer_drivers.hpp"
#include "aruwsrc/robot/hero/hero_drivers.hpp"
#include "aruwsrc/robot/sentry/sentry_drivers.hpp"
#include "aruwsrc/robot/standard/standard_drivers.hpp"

namespace aruwsrc
{
namespace control
{
#if defined(ALL_STANDARDS)
void initSubsystemCommands(aruwsrc::StandardDrivers *drivers);
#elif defined(TARGET_HERO_CYCLONE)
void initSubsystemCommands(aruwsrc::HeroDrivers *drivers);
#elif defined(TARGET_ENGINEER)
void initSubsystemCommands(aruwsrc::EngineerDrivers *drivers);
#elif defined(TARGET_DRONE)
void initSubsystemCommands(aruwsrc::DroneDrivers *drivers);
#elif defined(TARGET_SENTRY_BEEHIVE)
void initSubsystemCommands(aruwsrc::SentryDrivers *drivers);
#endif
}  // namespace control

}  // namespace aruwsrc

#endif  // ROBOT_CONTROL_HPP_
