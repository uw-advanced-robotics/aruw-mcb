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
    moveToOffsetPosition(
        drivers,
        turret,
        offsetConfig.pitchOffsetSteps,
        offsetConfig.yawOffsetSteps
    ),
    shouldMoveToOffsetPositionAfterLaunch(offsetConfig.shouldMoveToOffsetPositionAfterLaunch),
    timeDelayAfterLaunching(offsetConfig.timeDelayAfterLaunching)
    {
        addSubsystemRequirement(&turret);
        addSubsystemRequirement(&sled);

        comprisedCommandScheduler.registerSubsystem(&turret);
        comprisedCommandScheduler.registerSubsystem(&sled);
    }

    void LaunchDartComprisedCommand::initialize()
    {
        state = CommandState::LAUNCH_DART_ONE;
        comprisedCommandScheduler.addCommand(&moveSledToLaunchPosition);
    }

    void LaunchDartComprisedCommand::execute()
    {
        switch(state)
        {
            case CommandState::LAUNCH_DART_ONE:
            {
                if (!comprisedCommandScheduler.isCommandScheduled(&moveSledToLaunchPosition))
                {
                    if (shouldMoveToOffsetPositionAfterLaunch)
                    {
                        prevTime = tap::arch::clock::getTimeMilliseconds();
                        timeInWaiting = 0.0f;
                        state = CommandState::WAIT;
                    }
                    else
                    {
                        state = CommandState::FINISHED;
                    }
                }
                break;
            }
            case CommandState::WAIT:
            {
                float currentTime = tap::arch::clock::getTimeMilliseconds();
                if (currentTime - prevTime >= timeDelayAfterLaunching)
                {
                    comprisedCommandScheduler.addCommand(&moveToOffsetPosition);
                    state = CommandState::MOVE_TO_OFFSET_POSITION;
                }
                break;
            }
            case CommandState::MOVE_TO_OFFSET_POSITION:
            {
                break;
            }
        }
    }

    bool LaunchDartComprisedCommand::isFinished() const
    {
        return state == CommandState::FINISHED;
    }

    void LaunchDartComprisedCommand::end(bool interrupted)
    {
        comprisedCommandScheduler.removeCommand(&moveSledToLaunchPosition, interrupted);
        comprisedCommandScheduler.removeCommand(&moveToOffsetPosition, interrupted);
    }
}