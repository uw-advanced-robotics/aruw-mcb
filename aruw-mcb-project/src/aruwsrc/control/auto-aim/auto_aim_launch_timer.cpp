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
    aruwsrc::communication::serial::VisionCoprocessor* visionCoprocessor,
    aruwsrc::algorithms::CvBallisticsSolver* ballistics,
    const float maxSinglePlateHitFrequency)
    : agitatorTypicalDelayMicroseconds(agitatorTypicalDelayMicroseconds),
      visionCoprocessor(visionCoprocessor),
      ballistics(ballistics),
      maxSinglePlateHitFrequency(maxSinglePlateHitFrequency)
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

    auto ballisticsSolution = ballistics->computeTurretAimAngles();

    if (!ballisticsSolution.has_value())
    {
        return LaunchInclination::NO_TARGET;
    }

    if (!ballisticsSolution->shotWindowValid)
    {
        return LaunchInclination::UNGATED;
    }

    // If hitting the same plate multiple times requires a fire rate that's too high, switch to
    // shooting once per plate
    bool targetPlateCenters =
        ballisticsSolution->shotWindowHalfWidth * maxSinglePlateHitFrequency < 500'000;

    // If we want to shoot once per plate at the plate center, clamping the window start to the
    // center time means we'll only try to shoot the instant we think our shot will hit the center.
    // If the agitator was busy, it will fire as early as possible before the plate window ends,
    // shooting as close to the center as possible.
    // Note: If half the plate takes more time to travel across the aim line than it does for us to
    // fire one shot, this means we might still try to hit the trailing end of the plate. This would
    // only occur if the true shot rate exceeds the one used in the above entry condition
    uint64_t shotWindowStart = ballisticsSolution->shotWindowCenter -
                               (targetPlateCenters ? 0 : ballisticsSolution->shotWindowHalfWidth);
    uint64_t shotWindowEnd =
        ballisticsSolution->shotWindowCenter + ballisticsSolution->shotWindowHalfWidth;
    float timeOfFlightSeconds = ballisticsSolution->timeOfFlight;

    if (timeOfFlightSeconds <= 0 || timeOfFlightSeconds > MAX_ALLOWED_FLIGHT_TIME_SECS)
    {
        return LaunchInclination::GATED_DENY;
    }

    uint64_t now = tap::arch::clock::getTimeMicroseconds();
    uint64_t effectiveFireTime = now + this->agitatorTypicalDelayMicroseconds;

    bool inShotWindow = effectiveFireTime >= shotWindowStart && effectiveFireTime <= shotWindowEnd;
    if (!inShotWindow)
    {
        return LaunchInclination::GATED_DENY;
    }

    return LaunchInclination::GATED_ALLOW;
}
}  // namespace aruwsrc::control::auto_aim
