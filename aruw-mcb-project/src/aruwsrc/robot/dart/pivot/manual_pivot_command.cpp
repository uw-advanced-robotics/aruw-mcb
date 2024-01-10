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

#include "manual_pivot_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include <assert.h>

using namespace tap::communication::serial;
using namespace tap::algorithms;

namespace aruwsrc::robot::dart
{
ManualPivotCommand::ManualPivotCommand(
        tap::Drivers* drivers,
        PivotSubsystem* pivotSubsystem,
        RemoteInputChannel remoteInputChannel)
        : drivers(drivers),
          pivotSubsystem(pivotSubsystem),
          remoteInputChannel(remoteInputChannel)
{
    assert(pivotSubsystem != nullptr);
    addSubsystemRequirement(pivotSubsystem);
}

void ManualPivotCommand::execute()
{
    if (pivotSubsystem->allMotorsOnline())
    {
        pivotSubsystem->setMotor(drivers->remote.getChannel(remoteInputChannel)*MANUAL_DRIVE_SPEED_MAG);
    }
}

}  // namespace aruwsrc::robot::dart
