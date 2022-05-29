/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef MULTI_SHOT_HANDLER_HPP_
#define MULTI_SHOT_HANDLER_HPP_

#include "tap/control/hold_repeat_command_mapping.hpp"

#include "fire_rate_manager.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control::agitator
{
/**
 * Class that stores and allows the user to set some ShooterState. Possible shooter states include
 * single, 10 Hz, or 20 Hz full auto mode.
 *
 * The MultiShotHandler has an associated HoldRepeatCommandMapping. When the shooter state is set,
 * the HoldRepeatCommandMapping's `maxTiemsToSchedule` is updated. Furthermore, a this object
 * modifies the fire rate of a FireRateManager, which is used to determine the fire rate of the
 * launcher. Thus, assuming the HoldRepeatCommandMapping is associated with some command that
 * launches a projectile, this class will control the "state" of the shooter.
 */
class MultiShotHandler
{
public:
    /**
     * State of the shooting mechanism, how many times the associated command mapping should
     * reschedule the command when the mapping is met.
     */
    enum ShooterState : uint8_t
    {
        SINGLE = 0,
        FULL_AUTO_10HZ,
        FULL_AUTO_20HZ,
        NUM_SHOOTER_STATES,
    };

    /**
     * @param[in] commandMapping The HoldRepeatCommandMapping whose `maxTimesToSchedule` variable to
     * update.
     * @param[in] burstCount What to set `maxTimesToSchedule` to when in burst mode.
     */
    MultiShotHandler(
        tap::control::HoldRepeatCommandMapping &commandMapping,
        FireRateManager &manaulFireRateLimiter,
        int burstCount);

    void setShooterState(ShooterState state);

    ShooterState getShooterState() const { return state; }

private:
    tap::control::HoldRepeatCommandMapping &commandMapping;
    FireRateManager &manaulFireRateLimiter;
    const int burstCount;
    ShooterState state = SINGLE;
};

}  // namespace aruwsrc::control::agitator

#endif  // MULTI_SHOT_HANDLER_HPP_
