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

#include "multi_shot_cv_command_mapping.hpp"

namespace aruwsrc::control::agitator
{
MultiShotCvCommandMapping::MultiShotCvCommandMapping(
    tap::Drivers &drivers,
    tap::control::Command &launchCommand,
    const tap::control::RemoteMapState &rms,
    std::optional<ManualFireRateReselectionManager *> fireRateReselectionManager,
    governor::CvOnTargetGovernor &cvOnTargetGovernor,
    ConstantVelocityAgitatorCommand &command)
    : tap::control::HoldRepeatCommandMapping(&drivers, {&launchCommand}, rms, false),
      fireRateReselectionManager(fireRateReselectionManager),
      cvOnTargetGovernor(cvOnTargetGovernor),
      command(command)
{
}

void MultiShotCvCommandMapping::executeCommandMapping(const tap::control::RemoteMapState &currState)
{
    int timesToReschedule = 0;

    float fireRate = 0.0f;

    auto launchMode = cvOnTargetGovernor.inShotTimingMode() ? FULL_AUTO : this->launchMode;
    switch (launchMode)
    {
        case SINGLE:
            timesToReschedule = 1;
            fireRate = ManualFireRateReselectionManager::MAX_FIRERATE_RPS;
            command.enableConstantRotation(false);
            break;
        case NO_HEATING:
            timesToReschedule = -1;
            fireRate = getCurrentBarrelCoolingRate();
            command.enableConstantRotation(false);
            break;
        case FULL_AUTO_10HZ:
            timesToReschedule = -1;
            fireRate = 10;
            command.enableConstantRotation(false);
            break;
        case FULL_AUTO:
            timesToReschedule = -1;
            fireRate = ManualFireRateReselectionManager::MAX_FIRERATE_RPS;
            command.enableConstantRotation(true);
            break;
        default:
            assert(false);
            break;
    }

    if (fireRateReselectionManager.has_value())
    {
        fireRateReselectionManager.value()->setFireRate(fireRate);
    }

    setMaxTimesToSchedule(timesToReschedule);

    tap::control::HoldRepeatCommandMapping::executeCommandMapping(currState);

    if (!held)
    {
        command.enableConstantRotation(false);
    }
}

}  // namespace aruwsrc::control::agitator
