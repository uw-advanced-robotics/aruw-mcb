/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "launch_dart_comprised_command.hpp"

namespace aruwsrc::control::dart
{
    LaunchDartComprisedCommand::LaunchDartComprisedCommand(
        aruwsrc::Drivers *drivers,
        aruwsrc::control::turret::StepperTurretSubsystem& turret,
        aruwsrc::agitator::AgitatorSubsystem& sled,
        LaunchDartComprisedCommandConfig config,
        LaunchDartOffsetConfig offsetConfig
    ) : ComprisedCommand(drivers),
    drivers(drivers),
    turret(turret),
    sled(sled),
    moveSledToLaunchPosition(
        &sled,
        config.agitatorSetpoint,
        config.agitatorAngularSpeed,
        config.agitatorSetpointTolerance,
        true,
        true
    ),
    timeDelayAfterMoving(offsetConfig.timeDelayAfterMoving),
    moveToOffsetPosition(
        drivers,
        turret,
        offsetConfig.pitchOffsetSteps,
        offsetConfig.yawOffsetSteps
    )
    { }

    void LaunchDartComprisedCommand::initialize()
    {
        state = CommandState::LAUNCH_DART_ONE;
        comprisedCommandScheduler.addCommand(&moveSledToLaunchPosition);
    }

    void LaunchDartComprisedCommand::execute()
    {
        switch(state)
        {
            case LAUNCH_DART_ONE:
                if (!comprisedCommandScheduler.isCommandScheduled(&moveSledToLaunchPosition))
                {
                    comprisedCommandScheduler.addCommand(&moveToOffsetPosition);
                    state = MOVE_TO_OFFSET_POSITION;
                }
                break;
            case MOVE_TO_OFFSET_POSITION:
                if (!comprisedCommandScheduler.isCommandScheduled(&moveToOffsetPosition))
                {
                    prevTime = tap::arch::clock::getTimeMilliseconds();
                    timeInWaiting = 0.0f;
                    state = WAIT;
                }
                break;
            case WAIT:
                float currentTime = tap::arch::clock::getTimeMilliseconds();
                timeInWaiting += currentTime - prevTime;
                prevTime = currentTime;
                if (timeInWaiting >= timeDelayAfterMoving)
                {
                    state = FINISHED;
                }
                break;
            case FINISHED:
                break;
        }
    }

    bool LaunchDartComprisedCommand::isFinished() const
    {
        return state == FINISHED;
    }

    void LaunchDartComprisedCommand::end(bool interrupted)
    {
        comprisedCommandScheduler.removeCommand(&moveSledToLaunchPosition, interrupted);
        comprisedCommandScheduler.removeCommand(&moveToOffsetPosition, interrupted);
    }
}