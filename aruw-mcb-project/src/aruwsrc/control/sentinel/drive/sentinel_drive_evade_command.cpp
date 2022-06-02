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

#include <cassert>
#include <cstdlib>

#include "sentinel_drive_evade_command.hpp"
#include "sentinel_drive_subsystem.hpp"

#ifndef PLATFORM_HOSTED
using modm::platform::RandomNumberGenerator;
#endif

using tap::control::Subsystem;

namespace aruwsrc::control::sentinel::drive
{
SentinelDriveEvadeCommand::SentinelDriveEvadeCommand(
    SentinelDriveSubsystem* subsystem,
    float speedFraction)
    : sentinelDriveSubsystem(subsystem),
      speedFactor(speedFraction)
{
    addSubsystemRequirement(subsystem);
}

void SentinelDriveEvadeCommand::initialize()
{
#ifndef PLATFORM_HOSTED
    RandomNumberGenerator::enable();
#endif

    reverseDirection(LARGE_ARMOR_PLATE_WIDTH, MAX_TRAVERSE_DISTANCE);
}

void SentinelDriveEvadeCommand::execute()
{
    float absolutePosition = this->sentinelDriveSubsystem->absolutePosition();

    // If the robot has gone at least the required distance, generate a new random RPM and random
    // distance.
    if (abs(this->positionWhenDirectionChanged - absolutePosition) >= this->distanceToDrive)
    {
        reverseDirection(LARGE_ARMOR_PLATE_WIDTH, MAX_TRAVERSE_DISTANCE);
    }

    // reverse direction if close to the end of the rail
    reverseDirectionIfCloseToEnd(absolutePosition);
}

void SentinelDriveEvadeCommand::reverseDirectionIfCloseToEnd(float absolutePosition)
{
    static constexpr float HALF_RAIL_LENGTH = SentinelDriveSubsystem::RAIL_LENGTH / 2.0f;
    static constexpr float HALF_SENTINEL_LENGTH = SentinelDriveSubsystem::SENTINEL_LENGTH / 2.0f;

    float desiredDriveSpeed = this->sentinelDriveSubsystem->getDesiredRpm();

    // robot must move past the halfway point of rail.
    if ((nearStartOfRail(absolutePosition) && desiredDriveSpeed <= 0) ||
        (nearEndOfRail(absolutePosition) && desiredDriveSpeed >= 0))
    {
        float distanceFromCenter = abs(HALF_RAIL_LENGTH - HALF_SENTINEL_LENGTH - absolutePosition);

        float distanceFromFarRail =
            abs(SentinelDriveSubsystem::RAIL_LENGTH - SentinelDriveSubsystem::SENTINEL_LENGTH -
                absolutePosition);

        reverseDirection(distanceFromCenter, distanceFromFarRail);
    }
}

void SentinelDriveEvadeCommand::end(bool) { this->sentinelDriveSubsystem->setDesiredRpm(0); }

bool SentinelDriveEvadeCommand::isFinished() const { return false; }

void SentinelDriveEvadeCommand::reverseDirection(int32_t minDistance, int32_t maxDistance)
{
    this->positionWhenDirectionChanged = this->sentinelDriveSubsystem->absolutePosition();

    auto newDesiredRpm = getNewDesiredRpm(getMinDesiredRpm(), getMaxDesiredRpm());
    this->sentinelDriveSubsystem->setDesiredRpm(newDesiredRpm);

    this->distanceToDrive = getRandomIntegerBetweenBounds(minDistance, maxDistance);
}

uint32_t SentinelDriveEvadeCommand::getRandomInteger()
{
#ifndef PLATFORM_HOSTED
    if (RandomNumberGenerator::isReady())
    {
        return RandomNumberGenerator::getValue();
    }
    else
    {
        return 0;
    }
#else
    return 0;
#endif
}

int32_t SentinelDriveEvadeCommand::getRandomIntegerBetweenBounds(int32_t min, int32_t max) const
{
    assert(min <= max);

    uint32_t range = max - min;
    uint32_t randomValue = getRandomInteger();
    uint32_t randomValueWithinRange = randomValue % range;

    return static_cast<int32_t>(randomValueWithinRange) + min;
}

int32_t SentinelDriveEvadeCommand::getNewDesiredRpm(int32_t min, int32_t max) const
{
    int32_t randomValue = getRandomIntegerBetweenBounds(min, max);
    return copysign(randomValue, -this->sentinelDriveSubsystem->getDesiredRpm());
}

}  // namespace aruwsrc::control::sentinel::drive

#endif
