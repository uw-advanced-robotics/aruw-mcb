/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#if defined(TARGET_SENTRY_BEEHIVE)
#include "aruwsrc/drivers_singleton.hpp"

using namespace aruwsrc::sentry;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace sentry_control
{
/* define subsystems --------------------------------------------------------*/

/* define commands ----------------------------------------------------------*/

/* define command mappings --------------------------------------------------*/

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
}

/* register subsystems here -------------------------------------------------*/
void registerSentrySubsystems(Drivers *drivers)
{
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultSentryCommands(Drivers *)
{
}

/* add any starting commands to the scheduler here --------------------------*/
void startSentryCommands(Drivers *drivers)
{
}

/* register io mappings here ------------------------------------------------*/
void registerSentryIoMappings(Drivers *drivers)
{
}
}  // namespace sentry_control

namespace aruwsrc::sentry
{
void initSubsystemCommands(aruwsrc::sentry::Drivers *drivers)
{

}
}  // namespace aruwsrc::sentry

#endif
