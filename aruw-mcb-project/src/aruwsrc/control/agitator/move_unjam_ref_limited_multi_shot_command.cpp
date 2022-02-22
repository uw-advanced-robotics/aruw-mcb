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

#include "move_unjam_ref_limited_multi_shot_command.hpp"

#include "aruwsrc/drivers.hpp"

#include "agitator_subsystem.hpp"

using namespace tap::control::setpoint;

namespace aruwsrc::agitator
{
MoveUnjamRefLimitedMultiShotCommand::MoveUnjamRefLimitedMultiShotCommand(
    aruwsrc::Drivers *drivers,
    AgitatorSubsystem *agitator17mm,
    float agitatorRotateAngle,
    float maxUnjamRotateAngle,
    uint32_t rotateTime,
    bool heatLimiting,
    float heatLimitBuffer,
    int burstNum)
    : tap::control::ComprisedCommand(drivers),
      burstNum(burstNum),
      moveUnjamRefLimitedCommand(
          drivers,
          agitator17mm,
          agitatorRotateAngle,
          maxUnjamRotateAngle,
          rotateTime,
          heatLimiting,
          heatLimitBuffer)
{
    addSubsystemRequirement(agitator17mm);
    comprisedCommandScheduler.registerSubsystem(agitator17mm);
}

bool MoveUnjamRefLimitedMultiShotCommand::isReady() { return moveUnjamRefLimitedCommand.isReady(); }

void MoveUnjamRefLimitedMultiShotCommand::initialize()
{
    switch (shooterState)
    {
        case SINGLE:
            agitatorMovementsLeft = 1;
            break;
        case BURST:
            agitatorMovementsLeft = burstNum;
            break;
        case FULL_AUTO:
            agitatorMovementsLeft = -1;
            break;
        case NUM_SHOOTER_STATES:
        default:
            agitatorMovementsLeft = 0;
            break;
    }

    if (agitatorMovementsLeft != 0)
    {
        comprisedCommandScheduler.addCommand(&moveUnjamRefLimitedCommand);
    }
}

void MoveUnjamRefLimitedMultiShotCommand::execute()
{
    comprisedCommandScheduler.run();

    if (!comprisedCommandScheduler.isCommandScheduled(&moveUnjamRefLimitedCommand))
    {
        agitatorMovementsLeft -= agitatorMovementsLeft > 0 ? 1 : 0;
        if (agitatorMovementsLeft != 0)
        {
            comprisedCommandScheduler.addCommand(&moveUnjamRefLimitedCommand);
        }
    }
}

bool MoveUnjamRefLimitedMultiShotCommand::isFinished() const
{
    return moveUnjamRefLimitedCommand.isFinished() && (agitatorMovementsLeft == 0);
}

void MoveUnjamRefLimitedMultiShotCommand::end(bool interrupted)
{
    comprisedCommandScheduler.removeCommand(&moveUnjamRefLimitedCommand, interrupted);
}

}  // namespace aruwsrc::agitator
