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

#include "sentinel_drive_evade_command.hpp"
#include "sentinel_drive_subsystem.hpp"

#ifndef PLATFORM_HOSTED
using modm::platform::RandomNumberGenerator;
#endif

using tap::control::Subsystem;

namespace aruwsrc::control::sentinel::drive
{
SentinelDriveEvadeCommand::SentinelDriveEvadeCommand(SentinelDriveSubsystem* subsystem)
    : sentinelDriveSubsystem(subsystem)
{
    addSubsystemRequirement(dynamic_cast<Subsystem*>(subsystem));
}

void SentinelDriveEvadeCommand::initialize()
{
    prevAuto = true;
    randDistance = LARGE_ARMOR_PLATE_WIDTH;
}

void SentinelDriveEvadeCommand::execute()
{
    float newPos = sentinelDriveSubsystem->absolutePosition();
    // If the robot has just switched from full traverse mode or if the robot has gone at least the
    // required distance, generate a new random RPM and random distance.
    if (abs(positionWhenDirectionChanged - newPos) >= randDistance || prevAuto)
    {
        prevAuto = false;
        positionWhenDirectionChanged = sentinelDriveSubsystem->absolutePosition();

        uint32_t randVal = portableRandom();
        currentRPM = randVal % (MAX_RPM - MIN_RPM + 1) + MIN_RPM;

        uint32_t rand = portableRandom();
        randDistance =
            rand % (int)(MAX_DISTANCE - LARGE_ARMOR_PLATE_WIDTH + 1) + (int)LARGE_ARMOR_PLATE_WIDTH;

        float sentinelRPM = sentinelDriveSubsystem->getRpm();
        currentRPM = copysignf(currentRPM, -sentinelRPM);
    }

    // reverse direction if close to the end of the rail
    float curPos = sentinelDriveSubsystem->absolutePosition();
    if ((currentRPM < 0 && curPos < TURNAROUND_BUFFER) ||
        (currentRPM > 0 && curPos > SentinelDriveSubsystem::RAIL_LENGTH -
                                        SentinelDriveSubsystem::SENTINEL_LENGTH -
                                        TURNAROUND_BUFFER))
    {
        currentRPM = -currentRPM;
    }

    sentinelDriveSubsystem->setDesiredRpm(currentRPM);
}

void SentinelDriveEvadeCommand::end(bool) { sentinelDriveSubsystem->setDesiredRpm(0); }

bool SentinelDriveEvadeCommand::isFinished() const { return false; }

float SentinelDriveEvadeCommand::portableRandom()
{
#ifndef PLATFORM_HOSTED
    RandomNumberGenerator::enable();
    if (RandomNumberGenerator::isReady())
    {
        return RandomNumberGenerator::getValue();
    }
    else
    {
        return 0;
    }
#else
    return -1;
#endif
}

}  // namespace aruwsrc::control::sentinel::drive

#endif
