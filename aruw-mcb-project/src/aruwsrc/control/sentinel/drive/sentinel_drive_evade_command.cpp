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
    addSubsystemRequirement(dynamic_cast<Subsystem*>(subsystem));
}

void SentinelDriveEvadeCommand::initialize()
{
#ifndef PLATFORM_HOSTED
    RandomNumberGenerator::enable();
#endif

    changeDirection(
        MIN_RPM * speedFactor,
        MAX_RPM * speedFactor,
        LARGE_ARMOR_PLATE_WIDTH,
        MAX_TRAVERSE_DISTANCE);
}

void SentinelDriveEvadeCommand::execute()
{
    float newPos = sentinelDriveSubsystem->absolutePosition();
    // If the robot has gone at least the required distance, generate a new random RPM and random
    // distance.
    if (abs(positionWhenDirectionChanged - newPos) >= randDistance)
    {
        changeDirection(
            MIN_RPM * speedFactor,
            MAX_RPM * speedFactor,
            LARGE_ARMOR_PLATE_WIDTH,
            MAX_TRAVERSE_DISTANCE);
    }

    // reverse direction if close to the end of the rail, robot must move past the halfway point of
    // rail.
    float curPos = sentinelDriveSubsystem->absolutePosition();
    if ((currentRPM < 0 && curPos < TURNAROUND_BUFFER) ||
        (currentRPM > 0 && curPos > SentinelDriveSubsystem::RAIL_LENGTH -
                                        SentinelDriveSubsystem::SENTINEL_LENGTH -
                                        TURNAROUND_BUFFER))
    {
        changeDirection(
            MIN_RPM * speedFactor,
            MAX_RPM * speedFactor,
            SentinelDriveSubsystem::RAIL_LENGTH / 2 - TURNAROUND_BUFFER,
            SentinelDriveSubsystem::RAIL_LENGTH - 3 * TURNAROUND_BUFFER);
    }

    sentinelDriveSubsystem->setDesiredRpm(currentRPM);
}

void SentinelDriveEvadeCommand::end(bool) { sentinelDriveSubsystem->setDesiredRpm(0); }

bool SentinelDriveEvadeCommand::isFinished() const { return false; }

uint32_t SentinelDriveEvadeCommand::portableRandom()
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

void SentinelDriveEvadeCommand::changeDirection(
    int minRPM,
    int maxRPM,
    int64_t minDist,
    int64_t maxDist)
{
    positionWhenDirectionChanged = sentinelDriveSubsystem->absolutePosition();
    setCurrentRPM(minRPM, maxRPM);
    randDistance = getRandomVal(minDist, maxDist);
}

void SentinelDriveEvadeCommand::setCurrentRPM(int min, int max)
{
    currentRPM = getRandomVal(min, max);
    float sentinelRPM = sentinelDriveSubsystem->getDesiredRpm();
    currentRPM = copysignf(currentRPM, -sentinelRPM);
}

float SentinelDriveEvadeCommand::getRandomVal(int64_t min, int64_t max)
{
    uint32_t randVal = portableRandom();
    return int64_t(randVal) % (max - min + 1) + min;
}
}  // namespace aruwsrc::control::sentinel::drive

#endif
