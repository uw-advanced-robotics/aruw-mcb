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

#include "hero_agitator_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/command_scheduler.hpp"
#include "tap/control/setpoint/commands/move_unjam_comprised_command.hpp"

#include "aruwsrc/drivers.hpp"

#include "velocity_agitator_subsystem.hpp"

using namespace tap::control::setpoint;
using namespace tap::algorithms;

namespace aruwsrc
{
namespace agitator
{
HeroAgitatorCommand::HeroAgitatorCommand(
    aruwsrc::Drivers& drivers,
    const Config& config,
    VelocityAgitatorSubsystem& kickerAgitator,
    VelocityAgitatorSubsystem& waterwheelAgitator,
    const aruwsrc::control::launcher::FrictionWheelSubsystem& frictionWheels,
    tap::control::Command& kickerFireCommand,
    tap::control::Command& kickerLoadCommand,
    tap::control::Command& waterwheelLoadCommand)
    : tap::control::ComprisedCommand(&drivers),
      drivers(drivers),
      kickerAgitator(kickerAgitator),
      waterwheelAgitator(waterwheelAgitator),
      kickerFireCommand(kickerFireCommand),
      kickerLoadCommand(kickerLoadCommand),
      waterwheelLoadCommand(waterwheelLoadCommand),
      frictionWheels(frictionWheels),
      heatLimiting(config.heatLimiting),
      heatLimitBuffer(config.heatLimitBuffer)
{
    comprisedCommandScheduler.registerSubsystem(&kickerAgitator);
    comprisedCommandScheduler.registerSubsystem(&waterwheelAgitator);
    addSubsystemRequirement(&kickerAgitator);
    addSubsystemRequirement(&waterwheelAgitator);
}

bool HeroAgitatorCommand::isReady()
{
    const auto& robotData = drivers.refSerial.getRobotData();

    return !compareFloatClose(frictionWheels.getDesiredLaunchSpeed(), 0.0f, 1E-5) &&
           kickerAgitator.isOnline() && waterwheelAgitator.isOnline() &&
           !(drivers.refSerial.getRefSerialReceivingData() && heatLimiting &&
             (robotData.turret.heat42 + heatLimitBuffer > robotData.turret.heatLimit42));
}

bool HeroAgitatorCommand::isFinished() const
{
    return compareFloatClose(frictionWheels.getDesiredLaunchSpeed(), 0.0f, 1E-5) ||
           !waterwheelAgitator.isOnline() || !kickerAgitator.isOnline() || currState == FINISHED;
}

void HeroAgitatorCommand::initialize()
{
    // Limit switch is active low, so need to negate the reading.
    if (drivers.turretMCBCanComm.getLimitSwitchDepressed())
    {
        currState = SHOOTING;
        const auto& robotData = drivers.refSerial.getRobotData();
        startingHeat = drivers.refSerial.getRefSerialReceivingData() ? robotData.turret.heat42 : 0;
        comprisedCommandScheduler.addCommand(&kickerFireCommand);
    }
    else
    {
        beginLoading();
    }
}

void HeroAgitatorCommand::execute()
{
    switch (currState)
    {
        case SHOOTING:
            if ((drivers.refSerial.getRefSerialReceivingData() &&
                 startingHeat < drivers.refSerial.getRobotData().turret.heat42) ||
                !comprisedCommandScheduler.isCommandScheduled(&kickerFireCommand))
            {
                beginLoading();
            }
            break;
        case LOAD:
            if (drivers.turretMCBCanComm.getLimitSwitchDepressed())
            {
                currState = FINISHED;
            }
            else if (
                !comprisedCommandScheduler.isCommandScheduled(&kickerLoadCommand) &&
                !comprisedCommandScheduler.isCommandScheduled(&waterwheelLoadCommand))
            {
                beginLoading();
            }
            break;
        case FINISHED:
            break;
    }
    if (currState != FINISHED)
    {
        comprisedCommandScheduler.run();
    }
}

void HeroAgitatorCommand::end(bool interrupted)
{
    comprisedCommandScheduler.removeCommand(&kickerFireCommand, interrupted);
    comprisedCommandScheduler.removeCommand(&kickerLoadCommand, interrupted);
    comprisedCommandScheduler.removeCommand(&waterwheelLoadCommand, interrupted);
}

void HeroAgitatorCommand::beginLoading()
{
    currState = LOAD;
    comprisedCommandScheduler.addCommand(&kickerLoadCommand);
    comprisedCommandScheduler.addCommand(&waterwheelLoadCommand);
}

}  // namespace agitator

}  // namespace aruwsrc
