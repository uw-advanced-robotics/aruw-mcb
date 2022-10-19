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

#if defined(ALL_SENTRIES)

#include "sentry_full_traverse_command.hpp"

using namespace tap::arch::clock;

namespace aruwsrc::control::sentry::drive
{
SentryFullTraverseCommand::SentryFullTraverseCommand(SentryDriveSubsystem* subsystem)
    : prevTime(0),
      velocityTargetGenerator(0),
      subsystemSentryDrive(subsystem)
{
    addSubsystemRequirement(subsystem);
}

void SentryFullTraverseCommand::initialize()
{
    prevTime = getTimeMilliseconds();
    velocityTargetGenerator.reset(0);
    velocityTargetGenerator.setTarget(MAX_DESIRED_TRAVERSE_SPEED);
}

void SentryFullTraverseCommand::execute()
{
    float curPos = subsystemSentryDrive->getAbsolutePosition();
    // reverse direction if close to the end of the rail
    if (velocityTargetGenerator.getValue() < 0 && curPos < TURNAROUND_BUFFER)
    {
        velocityTargetGenerator.setTarget(MAX_DESIRED_TRAVERSE_SPEED);
    }
    else if (
        velocityTargetGenerator.getValue() > 0 && curPos > SentryDriveSubsystem::RAIL_LENGTH -
                                                               SentryDriveSubsystem::SENTRY_LENGTH -
                                                               TURNAROUND_BUFFER)
    {
        velocityTargetGenerator.setTarget(-MAX_DESIRED_TRAVERSE_SPEED);
    }
    // update chassis target velocity
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    velocityTargetGenerator.update(RAMP_SPEED * static_cast<float>(currTime - prevTime));
    prevTime = currTime;
    subsystemSentryDrive->setDesiredRpm(velocityTargetGenerator.getValue());
}

void SentryFullTraverseCommand::end(bool) { subsystemSentryDrive->setDesiredRpm(0.0f); }

bool SentryFullTraverseCommand::isFinished() const { return false; }
}  // namespace aruwsrc::control::sentry::drive

#endif
