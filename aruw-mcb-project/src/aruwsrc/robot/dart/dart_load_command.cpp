/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "dart_load_command.hpp"

namespace aruwsrc::robot::dart
{
void DartLoadCommand::execute()
{
    switch (currentState)
    {
        case UNSET:
            launcherPullSubsystem->setSetpoint(
                loadingPositionAndPullback.pullingPositionOfLauncher);
            if (launcherPullSubsystem->atSetpoint())
            {
                currentState = PULLED_TO_LOADER_BACK;
            }
            break;
        case PULLED_TO_LOADER_BACK:
            setLoaderMotor(
                loadingPositionAndPullback.loadingPosition,
                loadingPositionAndPullback.loaderPositionOfHook);
            if (loaderSubsystem->atSetpoint(loadingPositionAndPullback.loadingPosition))
            {
                currentState = LOADER_DOWN;
            }
            break;
        case LOADER_DOWN:
            launcherPullSubsystem->setSetpoint(loadingPositionAndPullback.pullingPositionOfHook);
            if (launcherPullSubsystem->atSetpoint())
            {
                currentState = HOOKED;
            }
            break;
        case HOOKED:
            setLoaderMotor(loadingPositionAndPullback.loadingPosition, 0);
            if (loaderSubsystem->atSetpoint(loadingPositionAndPullback.loadingPosition))
            {
                currentState = LOADER_UP;
            }
            break;
        case LOADER_UP:
            launcherPullSubsystem->setSetpoint(pullBackLocation());
            if (launcherPullSubsystem->atSetpoint())
            {
                currentState = PULLED_BACK_TO_FIRE;
            }
            break;
        default:
            break;
    }
}

}  // namespace aruwsrc::robot::dart
