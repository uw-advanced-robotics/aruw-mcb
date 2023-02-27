/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "aruwsrc/util_macros.hpp"

#ifdef TARGET_TESTBED

#include "tap/control/command_mapper.hpp"
#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/governor/governor_with_fallback_command.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/setpoint/commands/unjam_integral_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/drivers.hpp"

#ifdef PLATFORM_HOSTED
#include "tap/communication/can/can.hpp"
#endif

using namespace tap::control::setpoint;
using namespace tap::control::governor;
using namespace tap::control;
using namespace tap::communication::serial;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
aruwsrc::driversFunc drivers = aruwsrc::DoNotUse_getDrivers;

namespace testbed_control
{
// Safe disconnect function
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* register subsystems here -------------------------------------------------*/
void registerTestbedSubsystems(aruwsrc::Drivers *drivers)
{
}

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultTestbedCommands(aruwsrc::Drivers *)
{
}

/* add any starting commands to the scheduler here --------------------------*/
void startTestbedCommands(aruwsrc::Drivers *drivers)
{
}

/* register io mappings here ------------------------------------------------*/
void registerTestbedIoMappings(aruwsrc::Drivers *drivers)
{
}
}  // namespace testbed_control

namespace aruwsrc::control
{
void initSubsystemCommands(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &standard_control::remoteSafeDisconnectFunction);
    testbed_control::initializeSubsystems();
    testbed_control::registerStandardSubsystems(drivers);
    testbed_control::setDefaultStandardCommands(drivers);
    testbed_control::startStandardCommands(drivers);
    testbed_control::registerStandardIoMappings(drivers);
}
}  // namespace aruwsrc::control

#endif  // TARGET_TESTBED
