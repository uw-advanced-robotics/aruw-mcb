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

#include "tap/control/command_scheduler.hpp"
#include "tap/control/setpoint/commands/move_unjam_comprised_command.hpp"

#include "aruwsrc/drivers.hpp"

#include "agitator_subsystem.hpp"

using namespace tap::control::setpoint;

namespace aruwsrc
{
namespace agitator
{
HeroAgitatorCommand::HeroAgitatorCommand(
    aruwsrc::Drivers* drivers,
    AgitatorSubsystem* kickerAgitator,
    AgitatorSubsystem* waterwheelAgitator,
    float kickerShootRotateAngle,
    uint32_t kickerShootRotateTime,
    float kickerLoadRotateAngle,
    float waterwheelLoadRotateAngle,
    uint32_t loadRotateTime,
    float waterwheelMaxUnjamAngle,
    bool heatLimiting,
    uint16_t heatLimitBuffer)
    : tap::control::ComprisedCommand(drivers),
      kickerFireCommand(kickerAgitator, kickerShootRotateAngle, kickerShootRotateTime, 0, false),
      kickerLoadCommand(kickerAgitator, kickerLoadRotateAngle, loadRotateTime, 0, false),
      waterwheelLoadCommand(
          drivers,
          waterwheelAgitator,
          waterwheelLoadRotateAngle,
          waterwheelMaxUnjamAngle,
          loadRotateTime,
          0),
      drivers(drivers),
      kickerAgitator(kickerAgitator),
      waterwheelAgitator(waterwheelAgitator),
      heatLimiting(heatLimiting),
      heatLimitBuffer(heatLimitBuffer)
{
    this->comprisedCommandScheduler.registerSubsystem(kickerAgitator);
    this->comprisedCommandScheduler.registerSubsystem(waterwheelAgitator);
    this->addSubsystemRequirement(kickerAgitator);
    this->addSubsystemRequirement(waterwheelAgitator);
}

bool HeroAgitatorCommand::isReady()
{
    const auto& robotData = drivers->refSerial.getRobotData();

    return kickerAgitator->isOnline() && waterwheelAgitator->isOnline() &&
           !(drivers->refSerial.getRefSerialReceivingData() && heatLimiting &&
             (robotData.turret.heat42 + heatLimitBuffer > robotData.turret.heatLimit42));
}

bool HeroAgitatorCommand::isFinished() const { return currState == FINISHED; }

void HeroAgitatorCommand::initialize()
{
    if (!drivers->digital.read(LIMIT_SWITCH_PIN))
    {
        currState = SHOOTING;
        const auto& robotData = drivers->refSerial.getRobotData();
        startingHeat = robotData.turret.heat42;
        this->comprisedCommandScheduler.addCommand(&kickerFireCommand);
    }
    else
    {
        beginLoading();
    }
}

void HeroAgitatorCommand::end(bool interrupted)
{
    startingHeat = 0;
    currState = SHOOTING;

    this->comprisedCommandScheduler.removeCommand(&kickerFireCommand, interrupted);
    this->comprisedCommandScheduler.removeCommand(&kickerLoadCommand, interrupted);
    this->comprisedCommandScheduler.removeCommand(&waterwheelLoadCommand, interrupted);
}

void HeroAgitatorCommand::execute()
{
    const auto& robotData = drivers->refSerial.getRobotData();
    switch (currState)
    {
        case SHOOTING:
            if ((!drivers->refSerial.getRefSerialReceivingData() &&
                 startingHeat < robotData.turret.heat42) ||
                kickerFireCommand.isFinished())
            {
                beginLoading();
            }
            break;
        case LOAD:
            if (!drivers->digital.read(LIMIT_SWITCH_PIN))
            {
                currState = FINISHED;
            }
            else if (kickerLoadCommand.isFinished() && waterwheelLoadCommand.isFinished())
            {
                beginLoading();
            }
            break;
        case FINISHED:
            break;
    }
    if (currState != FINISHED)
    {
        this->comprisedCommandScheduler.run();
    }
}

void HeroAgitatorCommand::beginLoading()
{
    currState = LOAD;
    this->comprisedCommandScheduler.addCommand(&kickerLoadCommand);
    this->comprisedCommandScheduler.addCommand(&waterwheelLoadCommand);
}

}  // namespace agitator

}  // namespace aruwsrc
