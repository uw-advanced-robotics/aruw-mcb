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
      agressiveEvadeCommand(sentinelChassis, 1.0),
      passiveEvadeCommand(sentinelChassis, 0.5),
      moveToFarRightCommand(
          *sentinelChassis,
          SentinelDriveToSideCommand::SentinelRailSide::CLOSE_RAIL,
          MOVE_TO_RIGHT_DRIVE_SPEED_RPM)
{
    this->addSubsystemRequirement(sentinelChassis);
    this->comprisedCommandScheduler.registerSubsystem(sentinelChassis);
    this->agressiveEvadeTimer.restart(0);
}

void SentinelAutoDriveComprisedCommand::initialize() {}

void SentinelAutoDriveComprisedCommand::execute()
{
    if (!this->drivers->refSerial.getRefSerialReceivingData())
    {
        if (!this->userRequestDriveMovement)
        {
            if (!this->comprisedCommandScheduler.isCommandScheduled(&this->moveToFarRightCommand))
            {
                this->comprisedCommandScheduler.addCommand(&this->moveToFarRightCommand);
            }
        }
        else if (!this->comprisedCommandScheduler.isCommandScheduled(&this->passiveEvadeCommand))
        {
            this->comprisedCommandScheduler.addCommand(&this->passiveEvadeCommand);
        }
    }
    else
    {
        const auto &robotData = this->drivers->refSerial.getRobotData();

        if (robotData.currentHp == robotData.maxHp)
        {
            // move to right of rail when no damage taken
            if (!this->comprisedCommandScheduler.isCommandScheduled(&this->moveToFarRightCommand))
            {
                this->comprisedCommandScheduler.addCommand(&this->moveToFarRightCommand);
            }
        }
        else
        {
            // move agressively when taking damage
            if (robotData.receivedDps > AGGRESSIVE_EVADE_DPS_THRESHOLD &&
                !this->comprisedCommandScheduler.isCommandScheduled(&this->agressiveEvadeCommand))
            {
                this->comprisedCommandScheduler.addCommand(&this->agressiveEvadeCommand);

                this->agressiveEvadeTimer.restart(MIN_TIME_SPENT_AGGRESSIVELY_EVADING);
            }
            else if (
                compareFloatClose(robotData.receivedDps, 0.0f, 1E-5) &&
                this->agressiveEvadeTimer.isExpired())
            {
                if (!this->userRequestDriveMovement)
                {
                    if (!this->comprisedCommandScheduler.isCommandScheduled(
                            &this->moveToFarRightCommand))
                    {
                        this->comprisedCommandScheduler.addCommand(&this->moveToFarRightCommand);
                    }
                }
                if (!this->comprisedCommandScheduler.isCommandScheduled(&this->passiveEvadeCommand))
                {
                    this->comprisedCommandScheduler.addCommand(&this->passiveEvadeCommand);
                }
            }
        }
    }

    this->comprisedCommandScheduler.run();
}

void SentinelAutoDriveComprisedCommand::end(bool interrupted)
{
    this->comprisedCommandScheduler.removeCommand(&this->agressiveEvadeCommand, interrupted);
    this->comprisedCommandScheduler.removeCommand(&this->passiveEvadeCommand, interrupted);
    this->comprisedCommandScheduler.removeCommand(&this->moveToFarRightCommand, interrupted);
}

bool SentinelAutoDriveComprisedCommand::isFinished() const { return false; }

}  // namespace aruwsrc::control::sentinel::drive
