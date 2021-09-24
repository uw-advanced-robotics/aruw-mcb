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

#include "turret_world_relative_command.hpp"

#include "../turret_subsystem.hpp"

namespace aruwsrc::control::turret
{
TurretWorldRelativeCommand::TurretWorldRelativeCommand(
    tap::Drivers *drivers,
    TurretSubsystem *turretSubsystem,
    const chassis::ChassisSubsystem *chassisSubsystem)
    : tap::control::ComprisedCommand(drivers),
      turretWRChassisImuCommand(drivers, turretSubsystem, chassisSubsystem),
      turretWRTurretImuCommand(drivers, turretSubsystem, chassisSubsystem)
{
    comprisedCommandScheduler.registerSubsystem(turretSubsystem);
    addSubsystemRequirement(turretSubsystem);
}

bool TurretWorldRelativeCommand::isReady()
{
    return turretWRChassisImuCommand.isReady() || turretWRTurretImuCommand.isReady();
}

bool TurretWorldRelativeCommand::isFinished() const
{
    return turretWRChassisImuCommand.isFinished() && turretWRTurretImuCommand.isFinished();
}

void TurretWorldRelativeCommand::initialize()
{
    // Try and use turret IMU, otherwise default to chassis IMU.
    if (turretWRTurretImuCommand.isReady())
    {
        comprisedCommandScheduler.addCommand(&turretWRTurretImuCommand);
    }
    else
    {
        comprisedCommandScheduler.addCommand(&turretWRChassisImuCommand);
    }
}

void TurretWorldRelativeCommand::execute()
{
    // Re-initialize if no commands scheduled
    if (comprisedCommandScheduler.getAddedCommandBitmap() == 0)
    {
        initialize();
    }

    comprisedCommandScheduler.run();
}

void TurretWorldRelativeCommand::end(bool interrupted)
{
    comprisedCommandScheduler.removeCommand(&turretWRTurretImuCommand, interrupted);
    comprisedCommandScheduler.removeCommand(&turretWRChassisImuCommand, interrupted);
}

}  // namespace aruwsrc::control::turret
