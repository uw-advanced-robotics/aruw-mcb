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

#include "multi_shot_command_mapping.hpp"

#include "aruwsrc/control/turret/cv/turret_cv_command.hpp"
#include "aruwsrc/drivers.hpp"

namespace aruwsrc::agitator
{
MultiShotCommandMapping::MultiShotCommandMapping(
    aruwsrc::Drivers &drivers,
    tap::control::Command &singleLaunchCommand,
    tap::control::Command &fullAuto10HzCommand,
    tap::control::Command &fullAuto20HzCommand,
    const aruwsrc::control::turret::cv::TurretCVCommand &turretCvCommand,
    const tap::control::RemoteMapState &rms)
    : tap::control::HoldRepeatCommandMapping(&drivers, {&singleLaunchCommand}, rms, false, 1),
      drivers(drivers),
      singleLaunchCommand(singleLaunchCommand),
      fullAuto10HzCommand(fullAuto10HzCommand),
      fullAuto20HzCommand(fullAuto20HzCommand),
      turretCvCommand(turretCvCommand)
{
}

void MultiShotCommandMapping::setShooterState(ShooterState state) { this->state = state; }

void MultiShotCommandMapping::executeCommandMapping(const tap::control::RemoteMapState &currState)
{
    // TODO might be best for the governor to own some sort of "is active" flag that checks these
    // two conditions
    if (drivers.commandScheduler.isCommandScheduled(&turretCvCommand) /*&& cvGovernor.isEnabled()*/)
    {
        setMaxTimesToSchedule(getTimesToReschedule(FULL_AUTO_20HZ));
        mappedCommands[0] = &getLaunchCommand(FULL_AUTO_20HZ);
    }
    else
    {
        setMaxTimesToSchedule(getTimesToReschedule(this->state));
        mappedCommands[0] = &getLaunchCommand(this->state);
    }

    tap::control::HoldRepeatCommandMapping::executeCommandMapping(currState);
}

}  // namespace aruwsrc::agitator
