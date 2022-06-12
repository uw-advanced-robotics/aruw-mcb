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

#include "sentinel_drive_to_side_command.hpp"

#include <cassert>

namespace aruwsrc::control::sentinel::drive
{
SentinelDriveToSideCommand::SentinelDriveToSideCommand(
    SentinelDriveSubsystem &sentinelChassis,
    SentinelRailSide railSide,
    float movementSpeedRpm)
    : sentinelChassis(sentinelChassis),
      railSide(railSide),
      movementSpeedRpm(movementSpeedRpm)
{
    assert(movementSpeedRpm > 0.0f);
    addSubsystemRequirement(&sentinelChassis);
}

bool SentinelDriveToSideCommand::isReady()
{
    return this->sentinelChassis.allMotorsOnline() &&
           !this->withinRailEnd(this->railSide, this->sentinelChassis.getAbsolutePosition());
}

void SentinelDriveToSideCommand::initialize()
{
    float movementDirectionSign = this->railSide == SentinelRailSide::CLOSE_RAIL ? -1 : 1;
    this->sentinelChassis.setDesiredRpm(movementDirectionSign * movementSpeedRpm);
}

void SentinelDriveToSideCommand::execute() {}

void SentinelDriveToSideCommand::end(bool) { this->sentinelChassis.setDesiredRpm(0); }

bool SentinelDriveToSideCommand::isFinished() const
{
    return this->withinRailEnd(this->railSide, this->sentinelChassis.getAbsolutePosition());
}

}  // namespace aruwsrc::control::sentinel::drive
