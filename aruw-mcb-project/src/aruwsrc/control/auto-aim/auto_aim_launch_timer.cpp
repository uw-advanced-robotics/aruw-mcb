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

#include "auto_aim_launch_timer.hpp"

#include <tap/architecture/clock.hpp>

namespace aruwsrc::control::auto_aim
{
AutoAimLaunchTimer::AutoAimLaunchTimer(
    uint32_t agitatorTypicalDelayMicroseconds,
    aruwsrc::communication::serial::VisionCoprocessor *visionCoprocessor,
    aruwsrc::algorithms::CvBallisticsSolver *ballistics)
    : agitatorTypicalDelayMicroseconds(agitatorTypicalDelayMicroseconds),
      visionCoprocessor(visionCoprocessor),
      ballistics(ballistics)
{
}

AutoAimLaunchTimer::LaunchInclination AutoAimLaunchTimer::getCurrentLaunchInclination(
    uint8_t turretId)
{
    debugInfo = DebugInfo{};
    debugInfo.turretId = turretId;

    auto aimData = this->visionCoprocessor->getLastAimData(turretId);
    debugInfo.aimDataUpdated = aimData.pva.updated;
    debugInfo.timingDataUpdated = aimData.timing.updated;
    debugInfo.aimTimestamp = aimData.timestamp;
    debugInfo.pulseOffset = aimData.timing.offset;
    debugInfo.pulseInterval = aimData.timing.pulseInterval;
    debugInfo.pulseDuration = aimData.timing.duration;

    if (!aimData.pva.updated)
    {
        debugInfo.launchInclination = static_cast<uint8_t>(LaunchInclination::NO_TARGET);
        return LaunchInclination::NO_TARGET;
    }

    if (aimData.timing.updated && aimData.timing.pulseInterval == 0)
    {
        debugInfo.launchInclination = static_cast<uint8_t>(LaunchInclination::GATED_DENY);
        return LaunchInclination::GATED_DENY;
    }

    auto ballisticsSolution = ballistics->computeTurretAimAngles();
    debugInfo.ballisticsSolutionFound = ballisticsSolution.has_value();

    if (ballisticsSolution.has_value() && ballisticsSolution->usePulseEstimation)
    {
        float timeOfFlightSeconds = ballisticsSolution->timeOfFlight;
        debugInfo.pulseEstimationUsed = true;
        debugInfo.timeOfFlight = timeOfFlightSeconds;
        debugInfo.shotWindowStart = ballisticsSolution->shotWindowStart;
        debugInfo.shotWindowEnd = ballisticsSolution->shotWindowEnd;

        if (timeOfFlightSeconds <= 0 || timeOfFlightSeconds > MAX_ALLOWED_FLIGHT_TIME_SECS)
        {
            debugInfo.launchInclination = static_cast<uint8_t>(LaunchInclination::GATED_DENY);
            return LaunchInclination::GATED_DENY;
        }
        debugInfo.validFlightTime = true;

        uint64_t now = tap::arch::clock::getTimeMicroseconds();
        uint64_t effectiveFireTime = now + this->agitatorTypicalDelayMicroseconds;
        debugInfo.now = now;
        debugInfo.effectiveFireTime = effectiveFireTime;
        debugInfo.countdownToShotWindowStart =
            static_cast<int64_t>(ballisticsSolution->shotWindowStart) -
            static_cast<int64_t>(effectiveFireTime);
        debugInfo.countdownToShotWindowEnd =
            static_cast<int64_t>(ballisticsSolution->shotWindowEnd) -
            static_cast<int64_t>(effectiveFireTime);

        debugInfo.inShotWindow = effectiveFireTime >= ballisticsSolution->shotWindowStart &&
                                 effectiveFireTime <= ballisticsSolution->shotWindowEnd;
        if (debugInfo.inShotWindow)
        {
            debugInfo.launchInclination = static_cast<uint8_t>(LaunchInclination::GATED_ALLOW);
            return LaunchInclination::GATED_ALLOW;
        }
        else
        {
            debugInfo.launchInclination = static_cast<uint8_t>(LaunchInclination::GATED_DENY);
            return LaunchInclination::GATED_DENY;
        }
    }

    if (!aimData.timing.updated)
    {
        debugInfo.launchInclination = static_cast<uint8_t>(LaunchInclination::UNGATED);
        return LaunchInclination::UNGATED;
    }

    if (!ballisticsSolution.has_value())
    {
        debugInfo.launchInclination = static_cast<uint8_t>(LaunchInclination::GATED_DENY);
        return LaunchInclination::GATED_DENY;
    }

    float timeOfFlightSeconds = ballisticsSolution->timeOfFlight;
    debugInfo.timeOfFlight = timeOfFlightSeconds;
    if (timeOfFlightSeconds <= 0 || timeOfFlightSeconds > MAX_ALLOWED_FLIGHT_TIME_SECS)
    {
        debugInfo.launchInclination = static_cast<uint8_t>(LaunchInclination::GATED_DENY);
        return LaunchInclination::GATED_DENY;
    }
    debugInfo.validFlightTime = true;

    uint32_t timeOfFlightMicros = timeOfFlightSeconds * 1e6;
    uint32_t now = tap::arch::clock::getTimeMicroseconds();
    uint32_t projectedHitTime = now + this->agitatorTypicalDelayMicroseconds + timeOfFlightMicros;
    debugInfo.now = now;
    debugInfo.effectiveFireTime = projectedHitTime - timeOfFlightMicros;

    uint32_t nextPlateTransitTime = aimData.timestamp + aimData.timing.offset;
    int64_t projectedHitTimeAfterFirstWindow = projectedHitTime;
    projectedHitTimeAfterFirstWindow -= nextPlateTransitTime;

    int64_t pulseInterval = aimData.timing.pulseInterval;
    int64_t offsetInFiringWindow = projectedHitTimeAfterFirstWindow % pulseInterval;
    if (offsetInFiringWindow < 0)
    {
        offsetInFiringWindow += pulseInterval;
    }
    debugInfo.offsetInFiringWindow = offsetInFiringWindow;

    uint32_t maxHitTimeError = aimData.timing.duration / 2;
    debugInfo.maxHitTimeError = maxHitTimeError;
    debugInfo.inShotWindow = offsetInFiringWindow <= maxHitTimeError ||
                             offsetInFiringWindow >= pulseInterval - maxHitTimeError;
    if (debugInfo.inShotWindow)
    {
        debugInfo.launchInclination = static_cast<uint8_t>(LaunchInclination::GATED_ALLOW);
        return LaunchInclination::GATED_ALLOW;
    }
    debugInfo.launchInclination = static_cast<uint8_t>(LaunchInclination::GATED_DENY);
    return LaunchInclination::GATED_DENY;
}
}  // namespace aruwsrc::control::auto_aim
