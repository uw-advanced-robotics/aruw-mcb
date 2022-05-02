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

#include "move_cv_limited_command.hpp"

#include "aruwsrc/drivers.hpp"

namespace aruwsrc::agitator
{
MoveCVLimitedCommand::MoveCVLimitedCommand(
    aruwsrc::Drivers &drivers,
    tap::control::Subsystem &agitator,
    tap::control::Command &moveCommand,
    const aruwsrc::control::turret::cv::TurretCVCommand &turretCVCommand)
    : drivers(drivers),
      moveCommand(moveCommand),
      turretCVCommand(turretCVCommand)
{
    addSubsystemRequirement(&agitator);
}

bool MoveCVLimitedCommand::isReady() { return moveCommand.isReady() && visionAimingOnTarget(); }

bool MoveCVLimitedCommand::isFinished() const
{
    return moveCommand.isFinished() && !visionAimingOnTarget();
}

bool MoveCVLimitedCommand::visionAimingOnTarget() const
{
    return drivers.commandScheduler.isCommandScheduled(&turretCVCommand) &&
           turretCVCommand.isAimingWithinLaunchingTolerance();
}
}  // namespace aruwsrc::agitator