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
    std::optional<ManualFireRateReselectionManager*>
        fireRateReselectionManager,
    governor::CvOnTargetGovernor& cvOnTargetGovernor,
    std::optional<ConstantVelocityAgitatorCommand*> command)
    : drivers(drivers),
      launchCommand(launchCommand),
      singleShotCommand(&launchCommand, 1),
      repeatCommand(&launchCommand),
      fireRateReselectionManager(fireRateReselectionManager),
      cvOnTargetGovernor(cvOnTargetGovernor),
      command(command)
{
    commandRequirementsBitwise =
        launchCommand.getRequirementsBitwise();
}

void MultiShotCvCommand::setShooterState(
    LaunchMode mode)
{
    if (mode < NUM_SHOOTER_STATES)
    {
        launchMode = mode;
    }
}

MultiShotCvCommand::LaunchMode
MultiShotCvCommand::getLaunchMode() const
{
    return launchMode;
}

bool MultiShotCvCommand::isReady()
{
    return launchCommand.isReady();
}

void MultiShotCvCommand::initialize()
{
    initializedActiveCommand = false;
}

void MultiShotCvCommand::execute()
{
    float fireRate = 0.0f;
    bool enableConstantRotation = false;
    LaunchMode effectiveLaunchMode =
        cvOnTargetGovernor.inShotTimingMode()
            ? FULL_AUTO
            : launchMode;

    switch (effectiveLaunchMode)
    {
        case SINGLE:
            fireRate =
                ManualFireRateReselectionManager::
                    MAX_FIRERATE_RPS;

            activeCommand = &singleShotCommand;
            break;

        case NO_HEATING:
            fireRate =
                getCurrentBarrelCoolingRate();

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
            fireRate =
                ManualFireRateReselectionManager::
                    MAX_FIRERATE_RPS;

            enableConstantRotation = true;

            activeCommand = &repeatCommand;
            break;

        default:
            assert(false);
            return;
    }

    if (command.has_value())
    {
        command.value()->enableConstantRotation(
            enableConstantRotation);
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
    return false;
}

int MultiShotCvCommand::getCurrentBarrelCoolingRate() const
{
    int coolingRate =
        drivers.refSerial.getRobotData()
            .turret.coolingRate;

#if defined(TARGET_HERO_PERSEUS)
    return coolingRate / 100.0f;
#else
    return coolingRate / 10.0f;
#endif
}

}  // namespace aruwsrc::control::agitator