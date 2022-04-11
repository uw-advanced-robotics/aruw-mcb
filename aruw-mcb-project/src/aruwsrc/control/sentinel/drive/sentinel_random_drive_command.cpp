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

#if defined(TARGET_SENTINEL)

#ifndef PLATFORM_HOSTED
#include "modm/platform/random/random_number_generator.hpp"
#endif

#include "sentinel_drive_subsystem.hpp"
#include "sentinel_random_drive_command.hpp"

#ifndef PLATFORM_HOSTED
using modm::platform::RandomNumberGenerator;
#endif

using tap::control::Subsystem;

namespace aruwsrc::control::sentinel::drive
{
SentinelRandomDriveCommand::SentinelRandomDriveCommand(SentinelDriveSubsystem* subsystem)
    : subsystemSentinelDrive(subsystem),
      changeVelocityTimer(CHANGE_TIME_INTERVAL)
{
    addSubsystemRequirement(dynamic_cast<Subsystem*>(subsystem));

    // RandomNumberGenerator::enable();

}

void SentinelRandomDriveCommand::initialize() { 
    // chosenNewRPM = false;
    //oldPos = 0; 
    //newPos = 0;
    prevEvade = true;
    //currentRPM = 0;
}

void SentinelRandomDriveCommand::execute()
{

    if (this->changeVelocityTimer.isExpired() || prevEvade) 
    {

        //  chosenNewRPM = RandomNumberGenerator::isReady();

        newPos = subsystemSentinelDrive->absolutePosition();
        // if (chosenNewRPM && (abs(oldPos - newPos) >= 200.0 || prevEvade)) 
        if (abs(oldPos - newPos) >= 200.0 || prevEvade)
        {
            prevEvade = false;
            this->changeVelocityTimer.restart(CHANGE_TIME_INTERVAL);
            oldPos = subsystemSentinelDrive->absolutePosition();


            uint32_t randVal = portableRandom();
            currentRPM = randVal % (MAX_RPM - MIN_RPM + 1) + MIN_RPM;
            
            if (randVal % 2 == 0)
            {
                currentRPM *= -1.0f;
            } 
            
            float sentinelRPM = subsystemSentinelDrive->getRpm() ;
            if ((sentinelRPM < 0 && currentRPM < 0) || (sentinelRPM > 0 && currentRPM > 0))
            {
                currentRPM = -currentRPM;
            }
            
        } else {
            this->changeVelocityTimer.restart(CHANGE_TIME_INTERVAL);
        }
    }

    
    // reverse direction if close to the end of the rail
    float curPos = subsystemSentinelDrive->absolutePosition();
    if ((currentRPM < 0 && curPos < TURNAROUND_BUFFER) ||
        (currentRPM > 0 && curPos > SentinelDriveSubsystem::RAIL_LENGTH -
                                        SentinelDriveSubsystem::SENTINEL_LENGTH -
                                        TURNAROUND_BUFFER))
    {
        currentRPM = -currentRPM;
    }
    
    
    subsystemSentinelDrive->setDesiredRpm(currentRPM);
}

void SentinelRandomDriveCommand::end(bool) { subsystemSentinelDrive->setDesiredRpm(0); }

bool SentinelRandomDriveCommand::isFinished() const { return false; }

float SentinelRandomDriveCommand::portableRandom() 
{
    #ifndef PLATFORM_HOSTED
    RandomNumberGenerator::enable();
    if (RandomNumberGenerator::isReady())
    {
        return RandomNumberGenerator::getValue();
    } else {
        return 0;
    }
    #else
    return -1;    
    #endif
}

}  // namespace aruwsrc::control::sentinel::drive

#endif
