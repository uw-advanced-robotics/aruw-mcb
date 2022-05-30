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

#ifndef FIRE_RATE_MANAGER_HPP_
#define FIRE_RATE_MANAGER_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/util_macros.hpp"

#include "aruwsrc/control/governor/fire_rate_limit_governor.hpp"

namespace aruwsrc::control::agitator
{
/**
 * A storage class that contains a `fireRate` that can be set by calling `setFireRate`.
 */
class FireRateManager : public control::governor::FireRateManagerInterface
{
public:
    FireRateManager() {}

    mockable inline void setFireRate(float fireRate) { this->fireRate = std::max(0.0f, fireRate); }

    /// @return the set fire rate period (time distance between launching projectiles)
    inline uint32_t getFireRatePeriod() override { return rpsToPeriodMS(fireRate); }

    /**
     * Unless the fire rate is 0, always returns `FireRateReadinessState::READY_USE_RATE_LIMITING`
     * since the manager will always be ready to rate limit using the provided fireRate.
     */
    inline control::governor::FireRateReadinessState getFireRateReadinessState() override
    {
        if (tap::algorithms::compareFloatClose(fireRate, 0, 1E-5))
        {
            return control::governor::FireRateReadinessState::NOT_READY;
        }

        return control::governor::FireRateReadinessState::READY_USE_RATE_LIMITING;
    }

private:
    float fireRate = 0;
};
}  // namespace aruwsrc::control::agitator

#endif  // FIRE_RATE_MANAGER_HPP_
