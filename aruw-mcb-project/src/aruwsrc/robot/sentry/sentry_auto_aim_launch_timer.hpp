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

#ifndef AUTO_AIM_LAUNCH_TIMER_HPP_
#define AUTO_AIM_LAUNCH_TIMER_HPP_

#include <aruwsrc/algorithms/otto_ballistics_solver.hpp>
#include <aruwsrc/communication/serial/vision_coprocessor.hpp>
#include <aruwsrc/robot/sentry/sentry_ballistics_solver.hpp>

using namespace aruwsrc::sentry;

namespace aruwsrc::control::auto_aim
{
/**
 * A middleman between incoming vision coprocessor data and aim commands. Uses auto-aim obervations
 * to discriminate between four possible firing objectives at the current time. Chooses either
 * "no target" mode, indicating that there is no current target; "ungated" mode, which means no
 * further timing information is available; "gated allow", indicating that timing information
 * recommends firing right now; or "gated deny", which means that the vision system does not
 * currently recommend firing.
 *
 * Gating indicators are expected to vary at high frequency, as they use anticipated ballistics
 * time-of-flight and other delays to identify whether a projectile fired right now is expected to
 * hit a plate.
 */

class SentryAutoAimLaunchTimer
{
public:
    enum class LaunchInclination
    {
        NO_TARGET,
        UNGATED,
        GATED_DENY,
        GATED_ALLOW
    };
    static constexpr float MAX_ALLOWED_FLIGHT_TIME_SECS = 2.f;

private:
    uint32_t agitatorTypicalDelayMicroseconds;
    aruwsrc::serial::VisionCoprocessor *visionCoprocessor;
    SentryBallisticsSolver *ballistics;

public:
    SentryAutoAimLaunchTimer(
        uint32_t agitatorTypicalDelayMicroseconds,
        aruwsrc::serial::VisionCoprocessor *visionCoprocessor,
        SentryBallisticsSolver *ballistics);

    /**
     * Compute a firing inclination for the current time and specified turret.
     *
     * Uses the most recent aim data received for the chosen turret; if recency is desired,
     * callers should check whether the vision coprocessor is considered "online" before using this
     * method.
     *
     * @param[in] turretId the index of the desired turret.
     *
     * @return the computed LaunchInclination for time "now"
     */
    LaunchInclination getCurrentLaunchInclination(uint8_t turretId);

};  // class SentryAutoAimLaunchTimer

}  // namespace aruwsrc::control::auto_aim

#endif  // SENTRY_AUTO_AIM_LAUNCH_TIMER_HPP_
