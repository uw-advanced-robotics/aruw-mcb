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
#include <cstdlib>

#ifndef PLATFORM_HOSTED
using modm::platform::RandomNumberGenerator;
#endif

using tap::control::Subsystem;

namespace aruwsrc::control::sentinel::drive
{
SentinelDriveEvadeCommand::SentinelDriveEvadeCommand(SentinelDriveSubsystem* subsystem, float speed)
    : sentinelDriveSubsystem(subsystem)
{
    addSubsystemRequirement(dynamic_cast<Subsystem*>(subsystem));
    speedFactor = speed;
}

void SentinelDriveEvadeCommand::initialize()
{
    prevAuto = true;
    randDistance = LARGE_ARMOR_PLATE_WIDTH;
}

int counter = 0;
int counter2 = 0;
void SentinelDriveEvadeCommand::execute()
{
    float newPos = sentinelDriveSubsystem->absolutePosition();
    // If the robot has just switched from full traverse mode or if the robot has gone at least the
    // required distance, generate a new random RPM and random distance.
    if (abs(positionWhenDirectionChanged - newPos) >= randDistance || prevAuto)
    {
        counter++;
        prevAuto = false;
        positionWhenDirectionChanged = sentinelDriveSubsystem->absolutePosition();

        randomRPM(MIN_RPM*speedFactor, MAX_RPM*speedFactor);
        randDistance = (int)randomVal((int)LARGE_ARMOR_PLATE_WIDTH, (int)MAX_DISTANCE);
    }

    // reverse direction if close to the end of the rail
    float curPos = sentinelDriveSubsystem->absolutePosition();
    if ((currentRPM < 0 && curPos < TURNAROUND_BUFFER) ||
        (currentRPM > 0 && curPos > SentinelDriveSubsystem::RAIL_LENGTH -
                                        SentinelDriveSubsystem::SENTINEL_LENGTH -
                                        TURNAROUND_BUFFER))
    {
        
        counter2++;

        positionWhenDirectionChanged = sentinelDriveSubsystem->absolutePosition();
        randomRPM(MIN_RPM*speedFactor, MAX_RPM*speedFactor);
        randDistance = (int)randomVal((int)(SentinelDriveSubsystem::RAIL_LENGTH/2 - TURNAROUND_BUFFER), (int)(SentinelDriveSubsystem::RAIL_LENGTH - 3*TURNAROUND_BUFFER));
        
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

void SentinelDriveEvadeCommand::randomRPM(int min, int max)
{
    currentRPM = randomVal(min, max);
    float sentinelRPM = sentinelDriveSubsystem->getRpm();
    currentRPM = copysignf(currentRPM, -sentinelRPM);
}

float SentinelDriveEvadeCommand::randomVal(int min, int max)
{
    uint32_t randVal = portableRandom();
    return randVal % (max - min + 1) + min;
}
}  // namespace aruwsrc::control::sentinel::drive

#endif
