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
    aruwsrc::Drivers &drivers,
    tap::control::Command &launchCommand,
    const tap::control::RemoteMapState &rms,
    FireRateManager *fireRateManager,
    governor::CvOnTargetGovernor &cvOnTargetGovernor)
    : tap::control::HoldRepeatCommandMapping(&drivers, {&launchCommand}, rms, false),
      fireRateManager(fireRateManager),
      cvOnTargetGovernor(cvOnTargetGovernor)
{
}

void MultiShotHandler::executeCommandMapping(const tap::control::RemoteMapState &currState)
{
    int timesToReschedule = 0;

    float fireRate;
    switch (state)
    {
        case SINGLE:
            timesToReschedule = 1;
            fireRate = 1000;
            break;
        case FULL_AUTO_10HZ:
            timesToReschedule = -1;
            fireRate = 10;
            break;
        case FULL_AUTO:
            timesToReschedule = -1;
            fireRate = 1000;
            break;
        default:
            fireRate = 1000;
            break;
    }

    if (fireRateManager != nullptr)
    {
        fireRateManager->setFireRate(fireRate);
    }

    if (cvOnTargetGovernor.isGovernorGating())
    {
        timesToReschedule = -1;
    }

    setMaxTimesToSchedule(timesToReschedule);

    tap::control::HoldRepeatCommandMapping::executeCommandMapping(currState);
}
}  // namespace aruwsrc::control::agitator
