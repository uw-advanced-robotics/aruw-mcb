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

    // Get ballistics solution which contains pulse estimation timing windows
    auto ballisticsSolution = ballistics->computeTurretAimAngles();
    if (!ballisticsSolution.has_value())
    {
        return LaunchInclination::GATED_DENY;
    }

    // If not using pulse estimation (omega below threshold), fall back to ungated mode
    if (!ballisticsSolution->usePulseEstimation)
    {
        return LaunchInclination::UNGATED;
    }

    // Validate time of flight
    float timeOfFlightSeconds = ballisticsSolution->timeOfFlight;
    if (timeOfFlightSeconds <= 0 || timeOfFlightSeconds > MAX_ALLOWED_FLIGHT_TIME_SECS)
    {
        return LaunchInclination::GATED_DENY;
    }

    // Check if we're within the shot timing window
    // shotWindowStart and shotWindowEnd are absolute fire times (when to pull trigger)
    // accounting for time of flight, so we just need to check if now + agitator delay
    // is within the fire window
    uint64_t now = tap::arch::clock::getTimeMicroseconds();
    uint64_t effectiveFireTime = now + this->agitatorTypicalDelayMicroseconds;

    // Allow shot if the current time (accounting for agitator delay) is within the fire window
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
}  // namespace aruwsrc::control::auto_aim
