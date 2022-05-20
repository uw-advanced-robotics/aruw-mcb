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

#include <aruwsrc/communication/serial/vision_coprocessor.hpp>
#include <aruwsrc/algorithms/otto_ballistics_solver.hpp>

namespace aruwsrc::control::auto_aim
{
class AutoAimLaunchTimer
{
    public:
        enum class LaunchInclination { NO_TARGET, UNGATED, GATED_DENY, GATED_ALLOW };
        static constexpr float MAX_ALLOWED_FLIGHT_TIME_SECS = 2.f;

    private:
        uint32_t agitatorTypicalDelayMicroseconds;
        aruwsrc::serial::VisionCoprocessor *visionCoprocessor;
        aruwsrc::algorithms::OttoBallisticsSolver *ballistics;

        // std::optional<uint32_t> getTimedShotGoalTimestamp(uint8_t turretId);
    public:
        AutoAimLaunchTimer(uint32_t agitatorTypicalDelayMicroseconds, aruwsrc::serial::VisionCoprocessor *visionCoprocessor, aruwsrc::algorithms::OttoBallisticsSolver *ballistics);

        LaunchInclination getCurrentLaunchInclination(uint8_t turretId);

};  // class AutoAimLaunchTimer

}  // namespace aruwsrc::control::auto_aim

#endif  // AUTO_AIM_LAUNCH_TIMER_HPP_
