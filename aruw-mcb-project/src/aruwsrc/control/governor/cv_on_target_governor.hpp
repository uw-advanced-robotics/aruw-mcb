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
#include "aruwsrc/control/turret/cv/turret_cv_command_interface.hpp"
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
        tap::Drivers *drivers,
        aruwsrc::serial::VisionCoprocessor &visionCoprocessor,
        aruwsrc::control::turret::cv::TurretCVCommandInterface &turretCVCommand,
        AutoAimLaunchTimer &launchTimer,
        CvOnTargetGovernorMode mode)
        : drivers(drivers),
          visionCoprocessor(visionCoprocessor),
          turretCVCommand(turretCVCommand),
          launchTimer(launchTimer),
          mode(mode)
    {
    }

    mockable void setGovernorEnabled(bool enabled) { this->enabled = enabled; }

    mockable bool isGoverEnabled() const { return this->enabled; }

    /**
     * @return true if gating is being performed. If gating is being performed, projectiles will be
     * launched if the CV system decides they should be. The criteria are: 1. CV is onine and
     * connected. 2. the robot is executing the CV command. 3. CV Gating mode is enabled.
     * Otherwise, the system will not
     * be gated and projectiles may be launched independently of CV logic.
     */
    mockable bool isGovernorGating() const
    {
        bool isCvOnline = visionCoprocessor.isCvOnline();

        bool isCvRunning = drivers->commandScheduler.isCommandScheduled(&turretCVCommand);

        return isCvOnline && enabled && isCvRunning;
    }

    /**
     * @return true if gating is being performed and vision has deemed shot timing active.
     */
    mockable bool inShotTimingMode() const
    {
        bool gating = launchTimer.getCurrentLaunchInclination(turretCVCommand.getTurretID()) ==
                      AutoAimLaunchTimer::LaunchInclination::UNGATED;
        return isGovernorGating() && !gating;
    }

    bool isReady() final_mockable
    {
        if (!isGovernorGating())
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

    mockable bool isGateSatisfied()
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

    bool isFinished() final_mockable
    {
        // Once started, CV will not stop the target command; it is allowed to run to completion.
        // This enables firing a whole round, or burst, without interruption.
        return false;
    }

private:
    tap::Drivers *drivers;
    aruwsrc::serial::VisionCoprocessor &visionCoprocessor;
    aruwsrc::control::turret::cv::TurretCVCommandInterface &turretCVCommand;
    AutoAimLaunchTimer &launchTimer;
    const CvOnTargetGovernorMode mode;
    bool enabled = true;
};
}  // namespace aruwsrc::control::governor

#endif  // CV_ON_TARGET_GOVERNOR_HPP_