/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
    aruwsrc::serial::VisionCoprocessor *visionCoprocessor,
    aruwsrc::algorithms::OttoBallisticsSolver *ballistics
): agitatorTypicalDelayMicroseconds(agitatorTypicalDelayMicroseconds), visionCoprocessor(visionCoprocessor), ballistics(ballistics) {}

AutoAimLaunchTimer::LaunchInclination AutoAimLaunchTimer::getCurrentLaunchInclination(uint8_t turretId)
{
    auto aimData = this->visionCoprocessor->getLastAimData(turretId);
    if (!aimData.hasTarget) {
        return LaunchInclination::NO_TARGET;
    }

    if (!aimData.recommendUseTimedShots) {
        return LaunchInclination::UNGATED;
    }

    uint32_t now = tap::arch::clock::getTimeMicroseconds();
    uint32_t goalHitTime = aimData.timestamp + aimData.targetHitTimeOffset;

    float targetPitch;
    float targetYaw;
    float targetDistance;
    float timeOfFlightSeconds;
    // TODO: avoid re-solving
    bool ballisticsSolutionAvailable =
        this->ballistics->computeTurretAimAngles(&targetPitch, &targetYaw, &targetDistance, &timeOfFlightSeconds);

    if (!ballisticsSolutionAvailable) {
        return LaunchInclination::GATED_DENY;
    }

    if (timeOfFlightSeconds <= 0 || timeOfFlightSeconds > MAX_ALLOWED_FLIGHT_TIME_SECS) {
        return LaunchInclination::GATED_DENY;
    }

    uint32_t timeOfFlightMicros = timeOfFlightSeconds * 1e6;
    uint32_t projectedHitTime = now + this->agitatorTypicalDelayMicroseconds + timeOfFlightMicros;

    int64_t hitTimeError = goalHitTime - projectedHitTime;
    if (abs(hitTimeError) < aimData.targetIntervalDuration / 2) {
        return LaunchInclination::GATED_ALLOW;
    } else {
        return LaunchInclination::GATED_DENY;
    }
}
}
