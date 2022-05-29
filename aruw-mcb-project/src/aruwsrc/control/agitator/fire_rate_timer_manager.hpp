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

#ifndef FIRE_RATE_TIMER_MANAGER_HPP_
#define FIRE_RATE_TIMER_MANAGER_HPP_

#include "tap/architecture/clock.hpp"
#include "tap/architecture/timeout.hpp"

namespace aruwsrc::control::agitator
{
/**
 * A utility class that may be used to limit the fire rate vai checking if a projectile may be
 * launched given the time difference between the current time and when the last projectile was
 * fired. Check `isReadyToLaunchProjectile` before launching a projectile, then call
 * `setProjectileLaunched` once the projectile has been launched.
 */
class FireRateTimerManager
{
public:
    /**
     * This function must be called each time a projectile has been launched.
     *
     * @param[in] launchPeriod A period in milliseconds that projectiles will be limited to being
     * launched at. Projectiles can be launched at a longer period than the one specified, this is
     * simply an upper bound on the rate of fire.
     */
    inline void setProjectileLaunched(uint32_t newLaunchPeriod) { timer.restart(newLaunchPeriod); }

    /**
     * @return True if the manager deems that a projectile may be launched based on fire rate
     * limiting.
     */
    inline bool isReadyToLaunchProjectile() { return timer.isExpired() || timer.isStopped(); }

private:
    tap::arch::MilliTimeout timer;
};
}  // namespace aruwsrc::control::agitator

#endif  // FIRE_RATE_TIMER_MANAGER_HPP_
