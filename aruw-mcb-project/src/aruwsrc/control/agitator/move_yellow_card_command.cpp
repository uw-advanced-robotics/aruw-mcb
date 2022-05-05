/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "move_yellow_card_command.hpp"

#include <cassert>

#include "tap/control/subsystem.hpp"

#include "aruwsrc/drivers.hpp"

namespace aruwsrc::agitator
{
MoveYellowCardCommand::MoveYellowCardCommand(
    const aruwsrc::Drivers &drivers,
    tap::control::Subsystem &dependentSubsystem,
    tap::control::Command &normalCommand,
    tap::control::Command &yellowCardCommand)
    : drivers(drivers),
      normalCommand(normalCommand),
      yellowCardCommand(yellowCardCommand)
{
    assert(
        normalCommand.getRequirementsBitwise() == yellowCardCommand.getRequirementsBitwise() &&
        normalCommand.getRequirementsBitwise() ==
            (1UL << dependentSubsystem.getGlobalIdentifier()));
    addSubsystemRequirement(&dependentSubsystem);
}

bool MoveYellowCardCommand::isReady()
{
    const bool operatorBlinded = drivers.refSerial.operatorBlinded();
    return (operatorBlinded && yellowCardCommand.isReady()) ||
           (!operatorBlinded && normalCommand.isReady());
}

void MoveYellowCardCommand::initialize()
{
    if (drivers.refSerial.operatorBlinded())
    {
        yellowCardCommand.initialize();
        initializedWhenYellowCarded = true;
    }
    else
    {
        normalCommand.initialize();
        initializedWhenYellowCarded = false;
    }
}

void MoveYellowCardCommand::execute()
{
    if (initializedWhenYellowCarded)
    {
        yellowCardCommand.execute();
    }
    else
    {
        normalCommand.execute();
    }
}

void MoveYellowCardCommand::end(bool interrupted)
{
    if (initializedWhenYellowCarded)
    {
        yellowCardCommand.end(interrupted);
    }
    else
    {
        normalCommand.end(interrupted);
    }
}

bool MoveYellowCardCommand::isFinished() const
{
    return initializedWhenYellowCarded ? yellowCardCommand.isFinished()
                                       : normalCommand.isFinished();
}

}  // namespace aruwsrc::agitator
