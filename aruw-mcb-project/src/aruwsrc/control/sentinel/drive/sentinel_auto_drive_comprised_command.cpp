/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
      agressiveEvadeCommand(sentinelChassis, 0.3),
      passiveEvadeCommand(sentinelChassis, 0.7),
      moveToFarRightCommand(
          *sentinelChassis,
          SentinelDriveToSideCommand::SentinelRailSide::CLOSE_RAIL,
          MOVE_TO_RIGHT_DRIVE_SPEED_RPM)
{
    addSubsystemRequirement(sentinelChassis);
    this->comprisedCommandScheduler.registerSubsystem(sentinelChassis);
    this->agressiveEvadeTimer.stop();
}

void SentinelAutoDriveComprisedCommand::initialize() {}

void SentinelAutoDriveComprisedCommand::execute()
{
    if (!drivers->refSerial.getRefSerialReceivingData())
    {
        if (!this->comprisedCommandScheduler.isCommandScheduled(&this->passiveEvadeCommand))
        {
            this->comprisedCommandScheduler.addCommand(&this->passiveEvadeCommand);
        }
    }
    else
    {
        const auto &robotData = drivers->refSerial.getRobotData();

        if (robotData.maxHp == robotData.currentHp)
        {
            // move to right of rail when no damage taken
            this->comprisedCommandScheduler.removeCommand(&this->agressiveEvadeCommand, false);
            this->comprisedCommandScheduler.removeCommand(&this->passiveEvadeCommand, false);

            if (!this->comprisedCommandScheduler.isCommandScheduled(&this->moveToFarRightCommand))
            {
                this->comprisedCommandScheduler.addCommand(&this->moveToFarRightCommand);
            }
        }
        else
        {
            if (robotData.receivedDps > AGRESSIVE_EVADE_DPS_THRESHOLD)
            {
                if (!this->comprisedCommandScheduler.isCommandScheduled(
                        &this->agressiveEvadeCommand))
                {
                    this->comprisedCommandScheduler.addCommand(&this->agressiveEvadeCommand);

                    this->agressiveEvadeTimer.restart(MIN_TIME_SPENT_AGRESSIVELY_EVADING);
                }
            }
            else if (
                compareFloatClose(robotData.receivedDps, 0.0f, 1E-5) &&
                this->comprisedCommandScheduler.isCommandScheduled(&this->agressiveEvadeCommand) &&
                this->agressiveEvadeTimer.isExpired())
            {
                this->comprisedCommandScheduler.addCommand(&this->passiveEvadeCommand);
            }
        }
    }

    this->comprisedCommandScheduler.run();
}

void SentinelAutoDriveComprisedCommand::end(bool interrupted)
{
    this->comprisedCommandScheduler.removeCommand(&this->agressiveEvadeCommand, interrupted);
    this->comprisedCommandScheduler.removeCommand(&this->passiveEvadeCommand, interrupted);
}

bool SentinelAutoDriveComprisedCommand::isFinished() const { return false; }

}  // namespace aruwsrc::control::sentinel::drive

#endif
