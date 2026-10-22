/*
 * Copyright (c) 2020-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "aruwsrc/control/agitator/multi_shot_cv_command.hpp"

namespace aruwsrc::control::agitator
{
MultiShotCvCommand::MultiShotCvCommand(
    tap::Drivers& drivers,
    tap::control::Command& launchCommand,
    std::optional<ManualFireRateReselectionManager*> fireRateReselectionManager,
    governor::CvOnTargetGovernor& cvOnTargetGovernor,
    std::optional<ConstantVelocityAgitatorCommand*> command)
    : drivers(drivers),
      launchCommand(launchCommand),
      repeatCommand(&launchCommand),
      fireRateReselectionManager(fireRateReselectionManager),
      cvOnTargetGovernor(cvOnTargetGovernor),
      command(command)
{
    commandRequirementsBitwise = launchCommand.getRequirementsBitwise();
}

bool MultiShotCvCommand::isReady() { return launchCommand.isReady(); }

void MultiShotCvCommand::initialize()
{
    initializedActiveCommand = false;
    if (command.has_value()) command.value()->initialize();
}

void MultiShotCvCommand::execute()
{
    float fireRate = 0.0f;
    LaunchMode effectiveLaunchMode = cvOnTargetGovernor.inShotTimingMode() ? FULL_AUTO : launchMode;
    // for command state reset if launch mode changes while command is running
    switch (effectiveLaunchMode)
    {
        case SINGLE:
            fireRate = 1.0f;
            activeCommand = &launchCommand;
            break;
        case NO_HEATING:
            fireRate = getCurrentBarrelCoolingRate();
            activeCommand = &repeatCommand;
            break;
        case LIMITED_10HZ:
            fireRate = 10.0f;
            activeCommand = &repeatCommand;
            break;
        case LIMITED_20HZ:
            fireRate = 20.0f;
            activeCommand = &repeatCommand;
            break;
        case FULL_AUTO:
            fireRate = 30.0f;  // max firing rate
            activeCommand = &repeatCommand;
            break;
        default:
            break;
    }

    if (fireRateReselectionManager.has_value())
    {
        fireRateReselectionManager.value()->setFireRate(fireRate);
    }

    if (!initializedActiveCommand)
    {
        activeCommand->initialize();
        initializedActiveCommand = true;
    }

    activeCommand->execute();
}

void MultiShotCvCommand::end(bool interrupted)
{
    if (command.has_value())
    {
        command.value()->enableConstantRotation(false);
    }

    if (activeCommand != nullptr)
    {
        activeCommand->end(interrupted);
    }

    initializedActiveCommand = false;
}

bool MultiShotCvCommand::isFinished() const
{
    // Our expected behavior is that we fire one shot and the command ends, but since the trigger is
    // a whileTrue(), we keep the command continue not doing anything
    if (launchMode == SINGLE)
    {
        return false;
    }

    // Otherwise, see if the governor has determined that we should stop firing (e.g. due to heat or
    // CV)
    return launchCommand.isFinished();
}
}  // namespace aruwsrc::control::agitator