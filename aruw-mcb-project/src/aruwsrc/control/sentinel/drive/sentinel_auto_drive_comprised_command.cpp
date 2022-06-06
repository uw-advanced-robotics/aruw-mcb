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

#include "sentinel_auto_drive_comprised_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "aruwsrc/drivers.hpp"
#include "aruwsrc/util_macros.hpp"

#include "sentinel_drive_subsystem.hpp"

using namespace tap::algorithms;

namespace aruwsrc::control::sentinel::drive
{
SentinelAutoDriveComprisedCommand::SentinelAutoDriveComprisedCommand(
    aruwsrc::Drivers *drivers,
    SentinelDriveSubsystem *sentinelChassis)
    : tap::control::ComprisedCommand(drivers),
      drivers(drivers),
      aggressiveEvadeCommand(sentinelChassis, 1.0),
      passiveEvadeCommand(sentinelChassis, 0.5),
      moveToFarRightCommand(
          *sentinelChassis,
          SentinelDriveToSideCommand::SentinelRailSide::CLOSE_RAIL,
          MOVE_TO_RIGHT_DRIVE_SPEED_RPM)
{
    this->addSubsystemRequirement(sentinelChassis);
    this->comprisedCommandScheduler.registerSubsystem(sentinelChassis);
    this->aggressiveEvadeTimer.restart(0);
}

void SentinelAutoDriveComprisedCommand::initialize() {}

static inline void scheduleIfNotScheduled(
    tap::control::CommandScheduler &scheduler,
    tap::control::Command *cmd)
{
    if (!scheduler.isCommandScheduled(cmd))
    {
        scheduler.addCommand(cmd);
    }
}

void SentinelAutoDriveComprisedCommand::execute()
{
    if (!this->drivers->refSerial.getRefSerialReceivingData())
    {
        // not receiving ref serial data, toggle between passive evasion and moving to the far right
        // and stopping
        if (this->userRequestDriveMovement)
        {
            scheduleIfNotScheduled(this->comprisedCommandScheduler, &this->passiveEvadeCommand);
        }
        else
        {
            scheduleIfNotScheduled(this->comprisedCommandScheduler, &this->moveToFarRightCommand);
        }
    }
    else
    {
        const auto &robotData = this->drivers->refSerial.getRobotData();

        // override user requests to stop moving if agressively evading
        bool shouldMoveAgressively = robotData.receivedDps > AGGRESSIVE_EVADE_DPS_THRESHOLD;
        if (shouldMoveAgressively)
        {
            this->userRequestDriveMovement = true;
        }

        if (this->userRequestDriveMovement)
        {
            if (shouldMoveAgressively &&
                !this->comprisedCommandScheduler.isCommandScheduled(&this->aggressiveEvadeCommand))
            {
                this->comprisedCommandScheduler.addCommand(&this->aggressiveEvadeCommand);

                this->aggressiveEvadeTimer.restart(MIN_TIME_SPENT_AGGRESSIVELY_EVADING);
            }
            else if (
                compareFloatClose(robotData.receivedDps, 0.0f, 1E-5) &&
                this->aggressiveEvadeTimer.isExpired())
            {
                scheduleIfNotScheduled(this->comprisedCommandScheduler, &this->passiveEvadeCommand);
            }
        }
        else
        {
            scheduleIfNotScheduled(this->comprisedCommandScheduler, &this->moveToFarRightCommand);
        }
    }

    this->comprisedCommandScheduler.run();
}

void SentinelAutoDriveComprisedCommand::end(bool interrupted)
{
    this->comprisedCommandScheduler.removeCommand(&this->aggressiveEvadeCommand, interrupted);
    this->comprisedCommandScheduler.removeCommand(&this->passiveEvadeCommand, interrupted);
    this->comprisedCommandScheduler.removeCommand(&this->moveToFarRightCommand, interrupted);
}

bool SentinelAutoDriveComprisedCommand::isFinished() const { return false; }

}  // namespace aruwsrc::control::sentinel::drive
