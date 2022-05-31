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

#ifndef FIRE_RATE_TIMER_HPP_
#define FIRE_RATE_TIMER_HPP_

#include "tap/architecture/clock.hpp"

namespace aruwsrc::control::agitator
{
/**
 * A utility class that may be used to limit the fire rate via checking if a projectile may be
 * launched given the time difference between the current time and when the last projectile was
 * fired. Check `isReadyToLaunchProjectile` before launching a projectile, then call
 * `registerNewLaunchedProjectile` once the projectile has been launched.
 */
class FireRateTimer
{
public:
    /**
     * This function must be called each time a projectile has been launched.
     */
    inline void registerNewLaunchedProjectile()
    {
        this->lastProjectileLaunchTime = tap::arch::clock::getTimeMilliseconds();
    }

    /**
     * @return True if the manager deems that a projectile may be launched based on fire rate
     * limiting.
     */
    inline bool isReadyToLaunchProjectile() const
    {
        return tap::arch::clock::getTimeMilliseconds() - this->lastProjectileLaunchTime >=
               this->launchPeriod;
    }

    /**
     * Sets the projectile launch period.
     *
     * @param[in] launchPeriod A period in milliseconds that projectiles will be limited to being
     * launched at. Projectiles can be launched at a longer period than the one specified, this is
     * simply an upper bound on the rate of fire.
     */
    inline void setProjectileLaunchPeriod(uint32_t launchPeriod)
    {
        this->launchPeriod = launchPeriod;
    }

private:
    uint32_t lastProjectileLaunchTime = 0;
    uint32_t launchPeriod = 0;
};
}  // namespace aruwsrc::control::agitator

#endif  // FIRE_RATE_TIMER_HPP_
