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

#include "sentinel_drive_evade_command.hpp"

#include "aruwsrc/util_macros.hpp"

namespace aruwsrc::control::sentinel::drive
{
SentinelDriveEvadeCommand::SentinelDriveEvadeCommand(
    SentinelDriveSubsystem* subsystem,
    float speedFraction)
    : sentinelDriveSubsystem(subsystem),
      speedFactor(speedFraction)
{
    this->addSubsystemRequirement(subsystem);
}

void SentinelDriveEvadeCommand::initialize()
{
#ifndef PLATFORM_HOSTED
    modm::platform::RandomNumberGenerator::enable();
#endif

    this->reverseDirection(LARGE_ARMOR_PLATE_WIDTH, MAX_TRAVERSE_DISTANCE);
}

void SentinelDriveEvadeCommand::execute()
{
    float currentPosition = this->sentinelDriveSubsystem->absolutePosition();

    if (this->hasTraveledDistanceToDrive(currentPosition))
    {
        this->reverseDirection(LARGE_ARMOR_PLATE_WIDTH, MAX_TRAVERSE_DISTANCE);
    }

    this->reverseDirectionIfCloseToEnd(currentPosition);
}

void SentinelDriveEvadeCommand::reverseDirectionIfCloseToEnd(float currentPosition)
{
    static constexpr float HALF_RAIL_LENGTH = SentinelDriveSubsystem::RAIL_LENGTH / 2.0f;
    static constexpr float HALF_SENTINEL_LENGTH = SentinelDriveSubsystem::SENTINEL_LENGTH / 2.0f;

    float desiredDriveSpeed = this->sentinelDriveSubsystem->getDesiredRpm();

    if ((nearStartOfRail(currentPosition) && desiredDriveSpeed <= 0) ||
        (nearEndOfRail(currentPosition) && desiredDriveSpeed >= 0))
    {
        // robot must move past the halfway point of rail.
        float distanceFromCenter = abs(HALF_RAIL_LENGTH - HALF_SENTINEL_LENGTH - currentPosition);

        float distanceFromFarRail = 0;

        if (nearStartOfRail(currentPosition))
        {
            distanceFromFarRail =
                abs(SentinelDriveSubsystem::RAIL_LENGTH - SentinelDriveSubsystem::SENTINEL_LENGTH -
                    currentPosition);
        }
        else
        {
            distanceFromFarRail = abs(currentPosition);
        }

        this->reverseDirection(distanceFromCenter, distanceFromFarRail);
    }
}

void SentinelDriveEvadeCommand::end(bool) { this->sentinelDriveSubsystem->setDesiredRpm(0); }

bool SentinelDriveEvadeCommand::isFinished() const { return false; }

void SentinelDriveEvadeCommand::reverseDirection(int32_t minDistance, int32_t maxDistance)
{
    this->positionWhenDirectionChanged = this->sentinelDriveSubsystem->absolutePosition();

    auto newDesiredRpm = getNewDesiredRpm(
        getMinDesiredRpm(),
        getMaxDesiredRpm(),
        this->sentinelDriveSubsystem->getDesiredRpm());

    this->sentinelDriveSubsystem->setDesiredRpm(newDesiredRpm);

    this->distanceToDrive = getRandomIntegerBetweenBounds(minDistance, maxDistance);
}

}  // namespace aruwsrc::control::sentinel::drive
