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

#include "rotate_unjam_ref_limited_cv_gated_command.hpp"
#include "rotate_unjam_ref_limited_command.hpp"
#include "aruwsrc/control/auto-aim/auto_aim_launch_timer.hpp"
#include "aruwsrc/control/turret/cv/turret_cv_command.hpp"

#include "aruwsrc/drivers.hpp"

#include "agitator_subsystem.hpp"

using namespace tap::control::setpoint;
using namespace aruwsrc::control::auto_aim;

namespace aruwsrc::agitator
{
RotateUnjamRefLimitedCvGatedCommand::RotateUnjamRefLimitedCvGatedCommand(
    aruwsrc::Drivers &drivers,
    IntegrableSetpointSubsystem &subsystem,
    tap::control::setpoint::MoveIntegralCommand &moveIntegralCommand,
    tap::control::setpoint::UnjamIntegralCommand &unjamCommand,
    aruwsrc::control::turret::cv::TurretCVCommand &turretCVCommand,
    uint16_t heatLimitBuffer,
    aruwsrc::control::auto_aim::AutoAimLaunchTimer& autoAimLaunchTimer,
    uint8_t turretId)
    : RotateUnjamRefLimitedCommand(drivers, subsystem, moveIntegralCommand, unjamCommand, heatLimitBuffer),
      turretCVCommand(turretCVCommand),
      autoAimLaunchTimer(autoAimLaunchTimer),
      turretId(turretId)
{
}

bool RotateUnjamRefLimitedCvGatedCommand::isReady()
{
  const bool isParentReady = RotateUnjamRefLimitedCommand::isReady();
  if (!isParentReady) {
    return false;
  }

    if (drivers.commandScheduler.isCommandScheduled(&turretCVCommand)) {
        if (!turretCVCommand.isAimingWithinLaunchingTolerance()) {
            return false;
        }

        auto autoLaunchInclination = autoAimLaunchTimer.getCurrentLaunchInclination(turretId);
        switch (autoLaunchInclination) {
            case AutoAimLaunchTimer::LaunchInclination::NO_TARGET:
                return false;
            case AutoAimLaunchTimer::LaunchInclination::UNGATED:
                return true;
            case AutoAimLaunchTimer::LaunchInclination::GATED_ALLOW:
                return true;
            case AutoAimLaunchTimer::LaunchInclination::GATED_DENY:
                return false;
        }
    }

    return false;
}

}  // namespace aruwsrc::agitator
