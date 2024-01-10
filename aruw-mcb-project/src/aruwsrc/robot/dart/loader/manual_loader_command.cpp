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

#include "manual_loader_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include <assert.h>

using namespace tap::communication::serial;
using namespace tap::algorithms;

namespace aruwsrc::robot::dart
{
ManualLoaderCommand::ManualLoaderCommand(
    tap::Drivers* drivers,
    LoaderSubsystem* loaderSubsystem,
    RemoteInputSwitch magazineSelectionSwitch,
    RemoteInputChannel remoteInputChannel)
    : drivers(drivers),
      loaderSubsystem(loaderSubsystem),
      magazineSelectionSwitch(magazineSelectionSwitch),
      remoteInputChannel(remoteInputChannel)
{
    assert(loaderSubsystem != nullptr);
    addSubsystemRequirement(loaderSubsystem);
}

void ManualLoaderCommand::execute()
{
    commandIsRunning = true;
    motorsOnline = loaderSubsystem->allMotorsOnline();
    if (loaderSubsystem->allMotorsOnline())
    {
        int16_t velocitySetpoint = limitVal(
            drivers->remote.getChannel(remoteInputChannel) * LOAD_SPEED,
            LOAD_SPEED * -1.0f,
            LOAD_SPEED * 1.0f);

        switch (drivers->remote.getSwitch(magazineSelectionSwitch))
        {
            case tap::communication::serial::Remote::SwitchState::UP:
                loaderSubsystem->setMotors(velocitySetpoint, 0, 0);
                break;
            case tap::communication::serial::Remote::SwitchState::MID:
                loaderSubsystem->setMotors(0, velocitySetpoint, 0);
                break;
            case tap::communication::serial::Remote::SwitchState::DOWN:
                loaderSubsystem->setMotors(0, 0, velocitySetpoint);
                break;
            default:
                loaderSubsystem->setMotors(0, 0, 0);
                break;
        }
    }
}

}  // namespace aruwsrc::robot::dart