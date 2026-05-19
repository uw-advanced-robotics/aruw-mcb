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
#include "tap/drivers.hpp"

#include "aruwsrc/control/auto-aim/auto_aim_launch_timer.hpp"
#include "aruwsrc/control/turret/cv/turret_cv_command_interface.hpp"

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
        aruwsrc::communication::serial::VisionCoprocessor &visionCoprocessor,
        aruwsrc::control::turret::cv::TurretCVCommandInterface &turretCVCommand,
        AutoAimLaunchTimer &launchTimer,
        CvOnTargetGovernorMode mode,
        bool requireActiveGating = false)
        : drivers(drivers),
          visionCoprocessor(visionCoprocessor),
          turretCVCommand(turretCVCommand),
          launchTimer(launchTimer),
          mode(mode),
          requireActiveGating(requireActiveGating)
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

        bool gating = isCvOnline && enabled && isCvRunning;
        cvOnTargetDebugCvOnline = isCvOnline;
        cvOnTargetDebugCvRunning = isCvRunning;
        cvOnTargetDebugGovernorEnabled = enabled;
        cvOnTargetDebugGovernorGating = gating;
        return gating;
    }

    /**
     * @return true if gating is being performed and vision has deemed shot timing active.
     */
    mockable bool inShotTimingMode() const
    {
        auto launchInclination =
            launchTimer.getCurrentLaunchInclination(turretCVCommand.getTurretID());
        updateTimingDebug(launchInclination);
        bool ungated = launchInclination == AutoAimLaunchTimer::LaunchInclination::UNGATED;
        return isGovernorGating() && !ungated;
    }

    bool isReady() final_mockable
    {
        auto launchInclination =
            launchTimer.getCurrentLaunchInclination(turretCVCommand.getTurretID());
        updateTimingDebug(launchInclination);

        if (!isGovernorGating())
        {
            cvOnTargetDebugOnTarget = false;
            cvOnTargetDebugGateSatisfied = !requireActiveGating;
            cvOnTargetDebugIsReady = !requireActiveGating;
            return !requireActiveGating;
        }

        bool isOnTarget = turretCVCommand.isAimingWithinLaunchingTolerance();
        cvOnTargetDebugOnTarget = isOnTarget;
        if (!isOnTarget)
        {
            cvOnTargetDebugGateSatisfied = false;
            cvOnTargetDebugIsReady = false;
            return false;
        }

        cvOnTargetDebugIsReady = isGateSatisfied();
        return cvOnTargetDebugIsReady;
    }

    mockable bool isGateSatisfied()
    {
        auto autoLaunchInclination =
            launchTimer.getCurrentLaunchInclination(turretCVCommand.getTurretID());
        updateTimingDebug(autoLaunchInclination);

        bool gateSatisfied = false;
        switch (autoLaunchInclination)
        {
            case AutoAimLaunchTimer::LaunchInclination::NO_TARGET:
                gateSatisfied = false;
                break;
            case AutoAimLaunchTimer::LaunchInclination::UNGATED:
                gateSatisfied = true;
                break;
            case AutoAimLaunchTimer::LaunchInclination::GATED_ALLOW:
                gateSatisfied = true;
                break;
            case AutoAimLaunchTimer::LaunchInclination::GATED_DENY:
            {
                bool ignoreDenyGates = mode != CvOnTargetGovernorMode::ON_TARGET_AND_GATED;
                gateSatisfied = ignoreDenyGates;
                break;
            }
        }

        cvOnTargetDebugGateSatisfied = gateSatisfied;
        return gateSatisfied;
    }

    bool isFinished() final_mockable
    {
        // Once started, CV will not stop the target command; it is allowed to run to completion.
        // This enables firing a whole round, or burst, without interruption.
        return false;
    }

private:
    void updateTimingDebug(AutoAimLaunchTimer::LaunchInclination launchInclination) const
    {
        const auto &debugInfo = launchTimer.getDebugInfo();
        cvOnTargetDebugLaunchInclination = static_cast<uint8_t>(launchInclination);
        cvOnTargetDebugAimDataUpdated = debugInfo.aimDataUpdated;
        cvOnTargetDebugTimingDataUpdated = debugInfo.timingDataUpdated;
        cvOnTargetDebugBallisticsSolutionFound = debugInfo.ballisticsSolutionFound;
        cvOnTargetDebugPulseEstimationUsed = debugInfo.pulseEstimationUsed;
        cvOnTargetDebugValidFlightTime = debugInfo.validFlightTime;
        cvOnTargetDebugInShotWindow = debugInfo.inShotWindow;
        cvOnTargetDebugAimTimestamp = debugInfo.aimTimestamp;
        cvOnTargetDebugPulseOffset = debugInfo.pulseOffset;
        cvOnTargetDebugPulseInterval = debugInfo.pulseInterval;
        cvOnTargetDebugPulseDuration = debugInfo.pulseDuration;
        cvOnTargetDebugTimeOfFlight = debugInfo.timeOfFlight;
        cvOnTargetDebugNow = debugInfo.now;
        cvOnTargetDebugEffectiveFireTime = debugInfo.effectiveFireTime;
        cvOnTargetDebugShotWindowStart = debugInfo.shotWindowStart;
        cvOnTargetDebugShotWindowEnd = debugInfo.shotWindowEnd;
        cvOnTargetDebugCountdownToShotWindowStart = debugInfo.countdownToShotWindowStart;
        cvOnTargetDebugCountdownToShotWindowEnd = debugInfo.countdownToShotWindowEnd;
        cvOnTargetDebugOffsetInFiringWindow = debugInfo.offsetInFiringWindow;
        cvOnTargetDebugMaxHitTimeError = debugInfo.maxHitTimeError;
    }

    tap::Drivers *drivers;
    aruwsrc::communication::serial::VisionCoprocessor &visionCoprocessor;
    aruwsrc::control::turret::cv::TurretCVCommandInterface &turretCVCommand;
    AutoAimLaunchTimer &launchTimer;
    const CvOnTargetGovernorMode mode;
    const bool requireActiveGating;
    bool enabled = true;

    mutable bool cvOnTargetDebugCvOnline;
    mutable bool cvOnTargetDebugCvRunning;
    mutable bool cvOnTargetDebugGovernorEnabled;
    mutable bool cvOnTargetDebugGovernorGating;
    mutable bool cvOnTargetDebugOnTarget;
    mutable bool cvOnTargetDebugGateSatisfied;
    mutable bool cvOnTargetDebugIsReady;
    mutable uint8_t cvOnTargetDebugLaunchInclination;
    mutable bool cvOnTargetDebugAimDataUpdated;
    mutable bool cvOnTargetDebugTimingDataUpdated;
    mutable bool cvOnTargetDebugBallisticsSolutionFound;
    mutable bool cvOnTargetDebugPulseEstimationUsed;
    mutable bool cvOnTargetDebugValidFlightTime;
    mutable bool cvOnTargetDebugInShotWindow;
    mutable uint32_t cvOnTargetDebugAimTimestamp;
    mutable uint32_t cvOnTargetDebugPulseOffset;
    mutable uint32_t cvOnTargetDebugPulseInterval;
    mutable uint32_t cvOnTargetDebugPulseDuration;
    mutable float cvOnTargetDebugTimeOfFlight;
    mutable uint64_t cvOnTargetDebugNow;
    mutable uint64_t cvOnTargetDebugEffectiveFireTime;
    mutable uint64_t cvOnTargetDebugShotWindowStart;
    mutable uint64_t cvOnTargetDebugShotWindowEnd;
    mutable int64_t cvOnTargetDebugCountdownToShotWindowStart;
    mutable int64_t cvOnTargetDebugCountdownToShotWindowEnd;
    mutable int64_t cvOnTargetDebugOffsetInFiringWindow;
    mutable uint32_t cvOnTargetDebugMaxHitTimeError;
};
}  // namespace aruwsrc::control::governor

#endif  // CV_ON_TARGET_GOVERNOR_HPP_
