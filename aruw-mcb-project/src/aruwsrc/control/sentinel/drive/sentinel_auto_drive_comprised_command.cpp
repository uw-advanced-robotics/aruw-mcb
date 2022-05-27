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

#include "aruwsrc/util_macros.hpp"

#if defined(ALL_SENTINELS)

#include "tap/algorithms/math_user_utils.hpp"

#include "aruwsrc/drivers.hpp"

#include "sentinel_auto_drive_comprised_command.hpp"
#include "sentinel_drive_subsystem.hpp"

using namespace tap::algorithms;

namespace aruwsrc::control::sentinel::drive
{
SentinelAutoDriveComprisedCommand::SentinelAutoDriveComprisedCommand(
    aruwsrc::Drivers *drivers,
    SentinelDriveSubsystem *sentinelChassis)
    : tap::control::ComprisedCommand(drivers),
      drivers(drivers),
      sentinelChassis(sentinelChassis),
      evadeSlow(sentinelChassis, 0.5),
      evadeFast(sentinelChassis, 1.0),
      evadeMode(false)
{
    addSubsystemRequirement(sentinelChassis);
    comprisedCommandScheduler.registerSubsystem(sentinelChassis);
}

void SentinelAutoDriveComprisedCommand::initialize()
{
    comprisedCommandScheduler.addCommand(&evadeSlow);
}

void SentinelAutoDriveComprisedCommand::execute()
{
    const auto &robotData = drivers->refSerial.getRobotData();

    if (robotData.receivedDps > RANDOM_DRIVE_DPS_THRESHOLD)
    {
        if (!evadeMode)
        {
            comprisedCommandScheduler.removeCommand(&evadeSlow, true);
            comprisedCommandScheduler.addCommand(&evadeFast);
            evadeMode = true;
        }
    }
    else if (compareFloatClose(robotData.receivedDps, 0.0f, 1E-5) && evadeMode)
    {
        comprisedCommandScheduler.removeCommand(&evadeFast, true);
        comprisedCommandScheduler.addCommand(&evadeSlow);
        evadeMode = false;
    }

    comprisedCommandScheduler.run();
}

void SentinelAutoDriveComprisedCommand::end(bool interrupted)
{
    comprisedCommandScheduler.removeCommand(&evadeSlow, interrupted);
    comprisedCommandScheduler.removeCommand(&evadeFast, interrupted);
}

bool SentinelAutoDriveComprisedCommand::isFinished() const { return false; }

}  // namespace aruwsrc::control::sentinel::drive

#endif
