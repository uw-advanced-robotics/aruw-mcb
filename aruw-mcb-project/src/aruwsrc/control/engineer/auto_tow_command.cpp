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

#include "auto_tow_command.hpp"

namespace aruwsrc
{
namespace control
{
namespace engineer
{
AutoTowCommand::AutoTowCommand(TowSubsystem* subsystem) : towSubsystem(subsystem)
{
    this->addSubsystemRequirement(subsystem);
}

void AutoTowCommand::execute()
{
    if (!towSubsystem->getLeftClamped() && towSubsystem->getLeftLimitSwitchTriggered())
    {
        towSubsystem->setLeftClamped(true);
    }
    if (!towSubsystem->getRightClamped() && towSubsystem->getRightLeftLimitSwitchTriggered())
    {
        towSubsystem->setRightClamped(true);
    }
}

void AutoTowCommand::end(bool)
{
    towSubsystem->setLeftClamped(false);
    towSubsystem->setRightClamped(false);
}
}  // namespace engineer

}  // namespace control

}  // namespace aruwsrc
