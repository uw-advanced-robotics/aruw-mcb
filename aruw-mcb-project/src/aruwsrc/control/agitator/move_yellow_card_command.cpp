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

#include "aruwsrc/drivers.hpp"

namespace aruwsrc::agitator
{
MoveYellowCardCommand::MoveYellowCardCommand(
    aruwsrc::Drivers &drivers,
    tap::control::Subsystem &dependentSubsystem,
    tap::control::Command &moveCommandNormal,
    tap::control::Command &moveCommandWhenYellowCarded)
    : drivers(drivers),
      moveCommandNormal(moveCommandNormal),
      moveCommandWhenYellowCarded(moveCommandWhenYellowCarded)
{
    addSubsystemRequirement(&dependentSubsystem);
}

bool MoveYellowCardCommand::isReady()
{
    const bool operatorBlinded = drivers.refSerial.operatorBlinded();
    return (operatorBlinded && moveCommandWhenYellowCarded.isReady()) ||
           (!operatorBlinded && moveCommandNormal.isReady());
}

void MoveYellowCardCommand::initialize()
{
    if (drivers.refSerial.operatorBlinded())
    {
        moveCommandWhenYellowCarded.initialize();
        initializedWhenYellowCarded = true;
    }
    else
    {
        moveCommandNormal.initialize();
        initializedWhenYellowCarded = false;
    }
}
void MoveYellowCardCommand::execute()
{
    if (initializedWhenYellowCarded)
    {
        moveCommandWhenYellowCarded.execute();
    }
    else
    {
        moveCommandNormal.execute();
    }
}

void MoveYellowCardCommand::end(bool interrupted)
{
    if (initializedWhenYellowCarded)
    {
        moveCommandWhenYellowCarded.end(interrupted);
    }
    else
    {
        moveCommandNormal.end(interrupted);
    }
}

bool MoveYellowCardCommand::isFinished() const
{
    return initializedWhenYellowCarded ? moveCommandWhenYellowCarded.isFinished()
                                       : moveCommandNormal.isFinished();
}

}  // namespace aruwsrc::agitator
