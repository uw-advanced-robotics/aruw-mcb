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

#include "launch_darts_comprised_command.hpp"

namespace aruwsrc::control::dart
{
    LaunchDartsComprisedCommand::LaunchDartsComprisedCommand(
        aruwsrc::Drivers *drivers,
        aruwsrc::control::turret::StepperTurretSubsystem& turret,
        aruwsrc::agitator::AgitatorSubsystem& sled,
        LaunchDartsComprisedCommandConfig config
    ) : ComprisedCommand(drivers),
    drivers(drivers),
    turret(turret),
    sled(sled),
    moveSledToPositionOne(
        &sled,
        config.dartOneAgitatorSetpoint,
        config.agitatorAngularSpeed,
        config.agitatorSetpointTolerance,
        true,
        true
    ),
    moveSledToPositionTwo(
        &sled,
        config.dartTwoAgitatorSetpoint,
        config.agitatorAngularSpeed,
        config.agitatorSetpointTolerance,
        true,
        true
    ),
    moveToOffsetPosition(
        drivers,
        turret,
        config.pitchOffsetSteps,
        config.yawOffsetSteps
    )
    {
        state = CommandState::LAUNCH_DART_ONE;
    }

    void LaunchDartsComprisedCommand::initialize()
    {
        comprisedCommandScheduler.addCommand(&moveSledToPositionOne);
    }

    void LaunchDartsComprisedCommand::execute()
    {
        switch(state)
        {
            case LAUNCH_DART_ONE:
                
        }
    }

    bool LaunchDartsComprisedCommand::isFinished() const
    {

    }

    void LaunchDartsComprisedCommand::end(bool interrupted)
    {
        comprisedCommandScheduler.removeCommand(&moveSledToPositionOne, interrupted);
        comprisedCommandScheduler.removeCommand(&moveSledToPositionTwo, interrupted);
        comprisedCommandScheduler.removeCommand(&moveToOffsetPosition, interrupted);
    }
}