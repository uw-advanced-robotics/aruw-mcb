/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#if defined(TARGET_BLANK)

#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/blank/blank_drivers.hpp"
#include "aruwsrc/robot/robot_control.hpp"

using namespace aruwsrc::blank;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace blank_control
{
// Safe disconnect function
aruwsrc::control::RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

void initializeSubsystems() {}

}  // namespace blank_control

namespace aruwsrc::blank
{
void initSubsystemCommands(aruwsrc::blank::Drivers* drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &blank_control::remoteSafeDisconnectFunction);
    blank_control::initializeSubsystems();
}

}  // namespace aruwsrc::blank

#endif
