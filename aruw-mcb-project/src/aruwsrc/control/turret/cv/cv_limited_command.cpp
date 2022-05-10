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

#include "cv_limited_command.hpp"

#include "aruwsrc/drivers.hpp"

namespace aruwsrc::control::turret::cv
{
CVLimitedCommand::CVLimitedCommand(
    aruwsrc::Drivers &drivers,
    const std::vector<tap::control::Subsystem *> subsystemRequirements,
    tap::control::Command &command,
    const TurretCVCommand &turretCVCommand)
    : drivers(drivers),
      command(command),
      turretCVCommand(turretCVCommand)
{
    for (auto requirement : subsystemRequirements)
    {
        assert(requirement != nullptr);
        addSubsystemRequirement(requirement);
    }

    assert(command.getRequirementsBitwise() == this->getRequirementsBitwise());
}

bool CVLimitedCommand::isReady() { return command.isReady() && visionAimingOnTarget(); }

bool CVLimitedCommand::isFinished() const
{
    return command.isFinished() || !visionAimingOnTarget();
}

bool CVLimitedCommand::visionAimingOnTarget() const
{
    return drivers.commandScheduler.isCommandScheduled(&turretCVCommand) &&
           turretCVCommand.isAimingWithinLaunchingTolerance();
}
}  // namespace aruwsrc::control::turret::cv
