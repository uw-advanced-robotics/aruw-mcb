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

#ifndef FIRE_RATE_RESELECTION_MANAGER_INTERFACE_HPP_
#define FIRE_RATE_RESELECTION_MANAGER_INTERFACE_HPP_

#include <cmath>

namespace aruwsrc::control::agitator
{
enum class FireRateReadinessState
{
    NOT_READY = 0,
    READY_IGNORE_RATE_LIMITING,
    READY_USE_RATE_LIMITING,
};

/**
 * An interface that can be implemented to convey fire rate information to the
 * FireRateLimitGovernor.
 */
class FireRateReselectionManagerInterface
{
public:
    /// Max fire rate in rounds/second
    static constexpr float MAX_FIRERATE_RPS = 1000.0f;

    /// @return the fire rate period (time distance between launching projectiles)
    virtual uint32_t getFireRatePeriod() = 0;

    /// @return the readiness state of the
    virtual FireRateReadinessState getFireRateReadinessState() = 0;

protected:
    /**
     * Converts a rounds-per-second value (i.e., Hz) to a period in milliseconds.
     */
    static inline constexpr uint32_t rpsToPeriodMS(float rps)
    {
        if (rps == 0)
        {
            return UINT32_MAX;
        }
        else
        {
            return round(1000.0f / rps);
        }
    }
};
}  // namespace aruwsrc::control::agitator

#endif  // FIRE_RATE_RESELECTION_MANAGER_INTERFACE_HPP_
