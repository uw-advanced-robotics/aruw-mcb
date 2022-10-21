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

#include "sentry_drive_evade_command.hpp"

namespace aruwsrc::control::sentry::drive
{
SentryDriveEvadeCommand::SentryDriveEvadeCommand(
    SentryDriveSubsystem* sentryDriveSubsystem,
    float speedFraction)
    : sentryDriveSubsystem(sentryDriveSubsystem),
      speedFactor(speedFraction)
{
    this->addSubsystemRequirement(sentryDriveSubsystem);
}

void SentryDriveEvadeCommand::initialize()
{
#ifndef PLATFORM_HOSTED
    modm::platform::RandomNumberGenerator::enable();
#endif

    this->reverseDirectionForRandomDistance(LARGE_ARMOR_PLATE_WIDTH, MAX_TRAVERSE_DISTANCE);
}

void SentryDriveEvadeCommand::execute()
{
    float currentPosition = this->sentryDriveSubsystem->getAbsolutePosition();

    if (this->hasTraveledDistanceToDrive(currentPosition))
    {
        this->reverseDirectionForRandomDistance(LARGE_ARMOR_PLATE_WIDTH, MAX_TRAVERSE_DISTANCE);
    }

    this->reverseDirectionIfCloseToEnd(currentPosition);
}

void SentryDriveEvadeCommand::end(bool) { this->sentryDriveSubsystem->setDesiredRpm(0); }

bool SentryDriveEvadeCommand::isFinished() const { return false; }

void SentryDriveEvadeCommand::reverseDirectionForRandomDistance(
    int32_t minDistance,
    int32_t maxDistance)
{
    this->positionWhenDirectionChanged = this->sentryDriveSubsystem->getAbsolutePosition();

    auto newDesiredRpm = getNewDesiredRpm(
        getMinDesiredRpm(),
        getMaxDesiredRpm(),
        this->sentryDriveSubsystem->getDesiredRpm());

    this->sentryDriveSubsystem->setDesiredRpm(newDesiredRpm);

    this->distanceToDrive = getRandomIntegerBetweenBounds(minDistance, maxDistance);
}

void SentryDriveEvadeCommand::reverseDirectionIfCloseToEnd(float currentPosition)
{
    static constexpr float HALF_RAIL_LENGTH = SentryDriveSubsystem::RAIL_LENGTH / 2.0f;
    static constexpr float HALF_SENTRY_LENGTH = SentryDriveSubsystem::SENTRY_LENGTH / 2.0f;

    float desiredDriveSpeed = this->sentryDriveSubsystem->getDesiredRpm();

    if ((SentryDriveSubsystem::nearStartOfRail(currentPosition, TURNAROUND_BUFFER) &&
         desiredDriveSpeed <= 0) ||
        (SentryDriveSubsystem::nearEndOfRail(currentPosition, TURNAROUND_BUFFER) &&
         desiredDriveSpeed >= 0))
    {
        // robot must move past the halfway point of rail.
        float distanceFromCenter = abs(HALF_RAIL_LENGTH - HALF_SENTRY_LENGTH - currentPosition);

        float distanceFromFarRail = 0;

        if (SentryDriveSubsystem::nearStartOfRail(currentPosition, TURNAROUND_BUFFER))
        {
            distanceFromFarRail =
                abs(SentryDriveSubsystem::RAIL_LENGTH - SentryDriveSubsystem::SENTRY_LENGTH -
                    currentPosition);
        }
        else
        {
            distanceFromFarRail = abs(currentPosition);
        }

        this->reverseDirectionForRandomDistance(distanceFromCenter, distanceFromFarRail);
    }
}

}  // namespace aruwsrc::control::sentry::drive
