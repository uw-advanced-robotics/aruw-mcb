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

#include "turret_minor_sentry_world_relative_command.hpp"

#include "tap/drivers.hpp"

#include "../turret_subsystem.hpp"

namespace aruwsrc::control::turret::sentry
{
TurretMinorSentryWorldRelativeCommand::TurretMinorSentryWorldRelativeCommand(
    tap::Drivers *drivers,
    SentryControlOperatorInterface &controlOperatorInterface,
    SentryTurretMinorSubsystem *turretSubsystem,
    algorithms::TurretYawControllerInterface *chassisImuYawController,
    algorithms::TurretPitchControllerInterface *chassisImuPitchController,
    algorithms::TurretYawControllerInterface *turretImuYawController,
    algorithms::TurretPitchControllerInterface *turretImuPitchController,
    float userYawInputScalar,
    float userPitchInputScalar,
    uint8_t turretID)
    : tap::control::ComprisedCommand(drivers),
      turretWRChassisImuCommand(
          drivers,
          controlOperatorInterface,
          turretSubsystem,
          chassisImuYawController,
          chassisImuPitchController,
          userYawInputScalar,
          userPitchInputScalar,
          turretID),
      turretWRTurretImuCommand(
          drivers,
          controlOperatorInterface,
          turretSubsystem,
          turretImuYawController,
          turretImuPitchController,
          userYawInputScalar,
          userPitchInputScalar,
          turretID)
{
    comprisedCommandScheduler.registerSubsystem(turretSubsystem);
    addSubsystemRequirement(turretSubsystem);
}

bool TurretMinorSentryWorldRelativeCommand::isReady()
{
    return turretWRChassisImuCommand.isReady() || turretWRTurretImuCommand.isReady();
}

void TurretMinorSentryWorldRelativeCommand::initialize()
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

void TurretMinorSentryWorldRelativeCommand::execute()
{
    // Re-initialize if no commands scheduled or if the turret WR Turret IMU command
    // is ready and isn't scheduled
    if (comprisedCommandScheduler.getAddedCommandBitmap() == 0 ||
        (!comprisedCommandScheduler.isCommandScheduled(&turretWRTurretImuCommand) &&
         turretWRTurretImuCommand.isReady()))
    {
        initialize();
    }

    comprisedCommandScheduler.run();
}

bool TurretMinorSentryWorldRelativeCommand::isFinished() const
{
    return turretWRChassisImuCommand.isFinished() && turretWRTurretImuCommand.isFinished();
}

void TurretMinorSentryWorldRelativeCommand::end(bool interrupted)
{
    comprisedCommandScheduler.removeCommand(&turretWRTurretImuCommand, interrupted);
    comprisedCommandScheduler.removeCommand(&turretWRChassisImuCommand, interrupted);
}

}  // namespace aruwsrc::control::turret::sentry
