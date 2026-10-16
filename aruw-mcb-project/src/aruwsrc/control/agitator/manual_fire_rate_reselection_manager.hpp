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

#ifndef MANUAL_FIRE_RATE_RESELECTION_MANAGER_HPP_
#define MANUAL_FIRE_RATE_RESELECTION_MANAGER_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/util_macros.hpp"

#include "fire_rate_reselection_manager_interface.hpp"

namespace aruwsrc::control::agitator
{
/**
 * A container that stores a fire rate. The container will limit any specified fire rate to >= 0.
 * This object implements the FireRateReselectionManagerInterface and will be ready to fire whenever
 * the fire rate is > 0.
 */
class ManualFireRateReselectionManager
    : public control::agitator::FireRateReselectionManagerInterface
{
public:
    ManualFireRateReselectionManager() {}

    /// Sets fire rate and limits any negative fire rate to 0. Fire rate in rounds per second.
    mockable inline void setFireRate(float fireRate) { this->fireRate = std::max(0.0f, fireRate); }

    /// @return the set fire rate period (time distance between launching projectiles) in
    /// milliseconds
    inline uint32_t getFireRatePeriod() final_mockable { return rpsToPeriodMS(fireRate); }

    /**
     * Unless the fire rate is 0, always returns `FireRateReadinessState::READY_USE_RATE_LIMITING`
     * since the manager will always be ready to rate limit using the provided fireRate.
     */
    inline control::agitator::FireRateReadinessState getFireRateReadinessState() final_mockable
    {
        if (tap::algorithms::compareFloatClose(fireRate, 0, 1E-5))
        {
            return control::agitator::FireRateReadinessState::NOT_READY;
        }

        return control::agitator::FireRateReadinessState::READY_USE_RATE_LIMITING;
    }

private:
    float fireRate = 0;
};
}  // namespace aruwsrc::control::agitator

#endif  // MANUAL_FIRE_RATE_RESELECTION_MANAGER_HPP_
