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

#ifndef CV_ON_TARGET_GOVERNOR_HPP_
#define CV_ON_TARGET_GOVERNOR_HPP_

#include "tap/control/governor/command_governor_interface.hpp"

#include "aruwsrc/control/auto-aim/auto_aim_launch_timer.hpp"
#include "aruwsrc/control/turret/cv/turret_cv_command.hpp"
#include "aruwsrc/drivers.hpp"

namespace aruwsrc::control::governor
{
namespace
{
using namespace aruwsrc::control::auto_aim;
}

enum class CvOnTargetGovernorMode
{
    ON_TARGET,
    ON_TARGET_AND_GATED
};

/**
 * A governor that allows a Command to run when a TurretCVCommand has acquired and is aiming at a
 * target.
 */
class CvOnTargetGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    CvOnTargetGovernor(
        aruwsrc::Drivers &drivers,
        aruwsrc::control::turret::cv::TurretCVCommand &turretCVCommand,
        AutoAimLaunchTimer &launchTimer,
        CvOnTargetGovernorMode mode)
        : drivers(drivers),
          turretCVCommand(turretCVCommand),
          launchTimer(launchTimer),
          mode(mode)
    {
    }

    void setGovernorEnabled(bool enabled) { this->enabled = enabled; }
    bool getGovernorEnabled() const { return this->enabled; }

    bool isReady() final
    {
        if (!enabled)
        {
            return true;
        }

        bool isCvRunning = drivers.commandScheduler.isCommandScheduled(&turretCVCommand);
        if (!isCvRunning)
        {
            return true;
        }

        bool isOnTarget = turretCVCommand.isAimingWithinLaunchingTolerance();
        if (!isOnTarget)
        {
            return false;
        }

        return isGateSatisfied();
    }

    bool isGateSatisfied()
    {
        auto autoLaunchInclination =
            launchTimer.getCurrentLaunchInclination(turretCVCommand.getTurretID());
        switch (autoLaunchInclination)
        {
            case AutoAimLaunchTimer::LaunchInclination::NO_TARGET:
                return false;
            case AutoAimLaunchTimer::LaunchInclination::UNGATED:
                return true;
            case AutoAimLaunchTimer::LaunchInclination::GATED_ALLOW:
                return true;
            case AutoAimLaunchTimer::LaunchInclination::GATED_DENY:
                bool ignoreDenyGates = mode != CvOnTargetGovernorMode::ON_TARGET_AND_GATED;
                return ignoreDenyGates;
        }

        return false;
    }

    bool isFinished() final
    {
        // Once started, CV will not stop the target command; it is allowed to run to completion.
        // This enables firing a whole round, or burst, without interruption.
        return false;
    }

private:
    aruwsrc::Drivers &drivers;
    aruwsrc::control::turret::cv::TurretCVCommand &turretCVCommand;
    AutoAimLaunchTimer &launchTimer;
    const CvOnTargetGovernorMode mode;
    bool enabled = true;
};
}  // namespace aruwsrc::control::governor

#endif  // CV_ON_TARGET_GOVERNOR_HPP_