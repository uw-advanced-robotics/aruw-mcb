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

#include "multi_shot_handler.hpp"

namespace aruwsrc::control::agitator
{
MultiShotHandler::MultiShotHandler(
    tap::control::HoldRepeatCommandMapping &commandMapping,
    FireRateManager &manaulFireRateLimiter,
    int burstCount)
    : commandMapping(commandMapping),
      manaulFireRateLimiter(manaulFireRateLimiter),
      burstCount(burstCount)
{
}

void MultiShotHandler::setShooterState(ShooterState state)
{
    int timesToReschedule = 0;
    float fireRate;
    this->state = state;
    switch (state)
    {
        case SINGLE:
            timesToReschedule = 1;
            fireRate = 20;
            break;
        case FULL_AUTO_10HZ:
            timesToReschedule = -1;
            fireRate = 10;
            break;
        case FULL_AUTO_20HZ:
            timesToReschedule = -1;
            fireRate = 20;
            break;
        default:
            fireRate = 20;
            break;
    }

    commandMapping.setMaxTimesToSchedule(timesToReschedule);
    manaulFireRateLimiter.setFireRate(fireRate);
}
}  // namespace aruwsrc::control::agitator
