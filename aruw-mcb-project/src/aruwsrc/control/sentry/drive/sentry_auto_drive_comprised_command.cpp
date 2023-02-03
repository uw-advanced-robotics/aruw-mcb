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

#include "sentry_auto_drive_comprised_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/util_macros.hpp"

#include "sentry_drive_subsystem.hpp"

using namespace tap::algorithms;

namespace aruwsrc::control::sentry::drive
{
SentryAutoDriveComprisedCommand::SentryAutoDriveComprisedCommand(
    tap::Drivers *drivers,
    SentryDriveSubsystem *sentryChassis)
    : tap::control::ComprisedCommand(drivers),
      drivers(drivers),
      aggressiveEvadeCommand(sentryChassis, 1.0),
      passiveEvadeCommand(sentryChassis, 0.5),
      moveToFarRightCommand(
          *sentryChassis,
          SentryDriveToSideCommand::SentryRailSide::CLOSE_RAIL,
          MOVE_TO_RIGHT_DRIVE_SPEED_RPM)
{
    this->addSubsystemRequirement(sentryChassis);
    this->comprisedCommandScheduler.registerSubsystem(sentryChassis);
    this->aggressiveEvadeTimer.restart(0);
}

void SentryAutoDriveComprisedCommand::initialize() {}

static inline void scheduleIfNotScheduled(
    tap::control::CommandScheduler &scheduler,
    tap::control::Command *cmd)
{
    if (!scheduler.isCommandScheduled(cmd))
    {
        scheduler.addCommand(cmd);
    }
}

void SentryAutoDriveComprisedCommand::execute()
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

void SentryAutoDriveComprisedCommand::end(bool interrupted)
{
    this->comprisedCommandScheduler.removeCommand(&this->aggressiveEvadeCommand, interrupted);
    this->comprisedCommandScheduler.removeCommand(&this->passiveEvadeCommand, interrupted);
    this->comprisedCommandScheduler.removeCommand(&this->moveToFarRightCommand, interrupted);
}

bool SentryAutoDriveComprisedCommand::isFinished() const { return false; }

}  // namespace aruwsrc::control::sentry::drive
