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
    auto aimData = this->visionCoprocessor->getLastAimData(turretId);
    if (!aimData.pva.updated)
    {
        return LaunchInclination::NO_TARGET;
    }

    if (aimData.timing.updated && aimData.timing.pulseInterval == 0)
    {
        return LaunchInclination::GATED_DENY;
    }

    auto ballisticsSolution = ballistics->computeTurretAimAngles();

    if (ballisticsSolution.has_value() && ballisticsSolution->usePulseEstimation)
    {
        float timeOfFlightSeconds = ballisticsSolution->timeOfFlight;
        if (timeOfFlightSeconds <= 0 || timeOfFlightSeconds > MAX_ALLOWED_FLIGHT_TIME_SECS)
        {
            return LaunchInclination::GATED_DENY;
        }

        uint64_t now = tap::arch::clock::getTimeMicroseconds();
        uint64_t effectiveFireTime = now + this->agitatorTypicalDelayMicroseconds;

        if (effectiveFireTime >= ballisticsSolution->shotWindowStart &&
            effectiveFireTime <= ballisticsSolution->shotWindowEnd)
        {
            return LaunchInclination::GATED_ALLOW;
        }
        else
        {
            return LaunchInclination::GATED_DENY;
        }
    }

    if (!aimData.timing.updated)
    {
        return LaunchInclination::UNGATED;
    }

    if (!ballisticsSolution.has_value())
    {
        return LaunchInclination::GATED_DENY;
    }

    float timeOfFlightSeconds = ballisticsSolution->timeOfFlight;
    if (timeOfFlightSeconds <= 0 || timeOfFlightSeconds > MAX_ALLOWED_FLIGHT_TIME_SECS)
    {
        return LaunchInclination::GATED_DENY;
    }

    uint32_t timeOfFlightMicros = timeOfFlightSeconds * 1e6;
    uint32_t now = tap::arch::clock::getTimeMicroseconds();
    uint32_t projectedHitTime = now + this->agitatorTypicalDelayMicroseconds + timeOfFlightMicros;

    uint32_t nextPlateTransitTime = aimData.timestamp + aimData.timing.offset;
    int64_t projectedHitTimeAfterFirstWindow =
        static_cast<int64_t>(projectedHitTime) - static_cast<int64_t>(nextPlateTransitTime);

    int64_t pulseInterval = static_cast<int64_t>(aimData.timing.pulseInterval);
    int64_t offsetInFiringWindow = projectedHitTimeAfterFirstWindow % pulseInterval;
    if (offsetInFiringWindow < 0)
    {
        offsetInFiringWindow += pulseInterval;
    }

    int64_t maxHitTimeError = static_cast<int64_t>(aimData.timing.duration) / 2;
    if (offsetInFiringWindow <= maxHitTimeError ||
        offsetInFiringWindow >= pulseInterval - maxHitTimeError)
    {
        return LaunchInclination::GATED_ALLOW;
    }
    return LaunchInclination::GATED_DENY;
}
}  // namespace aruwsrc::control::auto_aim
