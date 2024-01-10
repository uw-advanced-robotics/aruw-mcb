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

#include "manual_launcher_pull_command.hpp"

#include <assert.h>

using namespace tap::communication::serial;

namespace aruwsrc::robot::dart
{
ManualLauncherPullCommand::ManualLauncherPullCommand(
    tap::Drivers* drivers,
    LauncherPullSubsystem* launcherPullSubsystem,
    RemoteInputChannel remoteInputChannel)
    : drivers(drivers),
      launcherPullSubsystem(launcherPullSubsystem),
      remoteInputChannel(remoteInputChannel)
{
    assert(launcherPullSubsystem != nullptr);
    addSubsystemRequirement(launcherPullSubsystem);
}

void ManualLauncherPullCommand::execute()
{
    if (launcherPullSubsystem->allMotorsOnline())
    {
        launcherPullSubsystem->setMotor(drivers->remote.getChannel(remoteInputChannel)*MAX_PULL_SPEED);
    }
}

}  // namespace aruwsrc::robot::dart
