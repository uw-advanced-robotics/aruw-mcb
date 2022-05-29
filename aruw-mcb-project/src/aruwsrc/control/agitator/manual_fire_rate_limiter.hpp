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

#ifndef MANUAL_FIRE_RATE_LIMITER_HPP_
#define MANUAL_FIRE_RATE_LIMITER_HPP_

#include <cstdint>

namespace aruwsrc::control::agitator
{
/**
 */
class ManualFireRateLimiter
{
public:
    ManualFireRateLimiter() {}

    void setFireRate(float fireRate) { this->fireRate = fireRate; }

    inline uint32_t getFireRatePeriod() { return rpsToPeriodMS(fireRate); }

    inline bool fireRateReady() { return true; }

private:
    float fireRate;

    /**
     * Converts a rounds-per-second value (i.e., Hz) to a period in milliseconds.
     */
    static inline constexpr uint32_t rpsToPeriodMS(float rps) { return (1000.0f / rps); }
};
}  // namespace aruwsrc::control::agitator

#endif  // MANUAL_FIRE_RATE_LIMITER_HPP_
