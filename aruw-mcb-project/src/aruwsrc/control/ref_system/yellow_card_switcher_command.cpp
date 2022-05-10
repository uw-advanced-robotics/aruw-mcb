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

#include "yellow_card_switcher_command.hpp"

#include <cassert>

#include "tap/control/subsystem.hpp"

#include "aruwsrc/drivers.hpp"

namespace aruwsrc::control::ref_system
{
YellowCardSwitcherCommand::YellowCardSwitcherCommand(
    const aruwsrc::Drivers &drivers,
    const std::vector<tap::control::Subsystem *> &subsystemRequirements,
    tap::control::Command &normalCommand,
    tap::control::Command &yellowCardCommand)
    : drivers(drivers),
      normalCommand(normalCommand),
      yellowCardCommand(yellowCardCommand)
{
    for (auto requirement : subsystemRequirements)
    {
        addSubsystemRequirement(requirement);
    }
    assert(normalCommand.getRequirementsBitwise() == this->getRequirementsBitwise());
    assert(yellowCardCommand.getRequirementsBitwise() == this->getRequirementsBitwise());
}

bool YellowCardSwitcherCommand::isReady()
{
    readyWhenYellowCarded = drivers.refSerial.operatorBlinded();
    return (readyWhenYellowCarded && yellowCardCommand.isReady()) ||
           (!readyWhenYellowCarded && normalCommand.isReady());
}

void YellowCardSwitcherCommand::initialize()
{
    if (readyWhenYellowCarded)
    {
        yellowCardCommand.initialize();
    }
    else
    {
        normalCommand.initialize();
    }
}

void YellowCardSwitcherCommand::execute()
{
    if (readyWhenYellowCarded)
    {
        yellowCardCommand.execute();
    }
    else
    {
        normalCommand.execute();
    }
}

void YellowCardSwitcherCommand::end(bool interrupted)
{
    if (readyWhenYellowCarded)
    {
        yellowCardCommand.end(interrupted);
    }
    else
    {
        normalCommand.end(interrupted);
    }
}

bool YellowCardSwitcherCommand::isFinished() const
{
    return readyWhenYellowCarded ? yellowCardCommand.isFinished() : normalCommand.isFinished();
}

}  // namespace aruwsrc::control::ref_system
