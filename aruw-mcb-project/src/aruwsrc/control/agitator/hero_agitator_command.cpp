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

#include <cassert>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/command_scheduler.hpp"
#include "tap/control/setpoint/commands/move_unjam_comprised_command.hpp"

#include "aruwsrc/drivers.hpp"
#include "aruwsrc/control/auto-aim/auto_aim_launch_timer.hpp"

#include "velocity_agitator_subsystem.hpp"

using namespace tap::control::setpoint;
using namespace tap::algorithms;

using namespace aruwsrc::control::auto_aim;

namespace aruwsrc
{
namespace agitator
{
HeroAgitatorCommand::HeroAgitatorCommand(
    aruwsrc::Drivers& drivers,
    const Config& config,
    tap::control::setpoint::IntegrableSetpointSubsystem& kickerAgitator,
    tap::control::setpoint::IntegrableSetpointSubsystem& waterwheelAgitator,
    const aruwsrc::control::launcher::FrictionWheelSubsystem& frictionWheels,
    const aruwsrc::control::turret::cv::TurretCVCommand& turretCVCommand,
    aruwsrc::control::auto_aim::AutoAimLaunchTimer& autoAimLaunchTimer,
    tap::control::Command& kickerFireCommand,
    tap::control::Command& kickerLoadCommand,
    tap::control::Command& waterwheelLoadCommand
    )
    : tap::control::ComprisedCommand(&drivers),
      drivers(drivers),
      kickerAgitator(kickerAgitator),
      waterwheelAgitator(waterwheelAgitator),
      kickerFireCommand(kickerFireCommand),
      kickerLoadCommand(kickerLoadCommand),
      waterwheelLoadCommand(waterwheelLoadCommand),
      frictionWheels(frictionWheels),
      turretCVCommand(turretCVCommand),
      autoAimLaunchTimer(autoAimLaunchTimer),
      heatLimiting(config.heatLimiting),
      heatLimitBuffer(config.heatLimitBuffer)
{
    uint64_t kickerBitwise = 1UL << kickerAgitator.getGlobalIdentifier();
    uint64_t waterwheelBitwise = 1UL << waterwheelAgitator.getGlobalIdentifier();
    assert(
        kickerBitwise == kickerFireCommand.getRequirementsBitwise() &&
        kickerBitwise == kickerLoadCommand.getRequirementsBitwise() &&
        waterwheelBitwise == waterwheelLoadCommand.getRequirementsBitwise());

    comprisedCommandScheduler.registerSubsystem(&kickerAgitator);
    comprisedCommandScheduler.registerSubsystem(&waterwheelAgitator);
    addSubsystemRequirement(&kickerAgitator);
    addSubsystemRequirement(&waterwheelAgitator);
}

static inline bool flywheelsOn(
    const aruwsrc::control::launcher::FrictionWheelSubsystem& frictionWheels)
{
    return !compareFloatClose(frictionWheels.getDesiredLaunchSpeed(), 0.0f, 1E-5);
}

static inline bool readyToFire(aruwsrc::Drivers& drivers)
{
    return drivers.turretMCBCanComm.getLimitSwitchDepressed();
}

static inline bool enoughHeatToFire(
    aruwsrc::Drivers& drivers,
    bool heatLimiting,
    const tap::communication::serial::RefSerialData::Rx::RobotData& robotData,
    uint16_t heatLimitBuffer)
{
    return !drivers.refSerial.getRefSerialReceivingData() || !heatLimiting ||
           (robotData.turret.heat42 + heatLimitBuffer <= robotData.turret.heatLimit42);
}

static inline bool readyToRotate(
    aruwsrc::Drivers& drivers,
    const aruwsrc::control::turret::cv::TurretCVCommand& turretCVCommand,
    aruwsrc::control::auto_aim::AutoAimLaunchTimer& autoAimLaunchTimer)
{
    const bool readyToFire = drivers.turretMCBCanComm.getLimitSwitchDepressed();

    /**
     * - If not ready to fire, we can rotate the agitators since we won't launch a projectile
     * - If the operator is not blinded, don't gate rotation
     * - If the turret CV is not scheduled, also don't gate rotation
     * - If the turret CV command reports its okay to fire, you can rotate to launch a projectile
     */
    if (!readyToFire) {
        return true;
    }
    
    // if (!drivers.refSerial.operatorBlinded()) {
    //     return true;
    // }

    if (drivers.commandScheduler.isCommandScheduled(&turretCVCommand)) {
        if (!turretCVCommand.isAimingWithinLaunchingTolerance()) {
            return false;
        }

        auto autoLaunchInclination = autoAimLaunchTimer.getCurrentLaunchInclination(0);
        switch (autoLaunchInclination) {
            case AutoAimLaunchTimer::LaunchInclination::NO_TARGET:
                return false;
            case AutoAimLaunchTimer::LaunchInclination::UNGATED:
                return true;
            case AutoAimLaunchTimer::LaunchInclination::GATED_ALLOW:
                return true;
            case AutoAimLaunchTimer::LaunchInclination::GATED_DENY:
                return false;
        }
    }

    // Normal human control
    return true;
}

bool HeroAgitatorCommand::isReady()
{
    const auto& robotData = drivers.refSerial.getRobotData();

    lastAutoLaunchInclination = autoAimLaunchTimer.getCurrentLaunchInclination(0); // TODO: testing

    return kickerAgitator.isOnline() && waterwheelAgitator.isOnline() &&
           flywheelsOn(frictionWheels) &&
           enoughHeatToFire(drivers, heatLimiting, robotData, heatLimitBuffer) &&
           readyToRotate(drivers, turretCVCommand, autoAimLaunchTimer);
}

bool HeroAgitatorCommand::isFinished() const
{
    return !flywheelsOn(frictionWheels) || !waterwheelAgitator.isOnline() ||
           !kickerAgitator.isOnline() || currState == FINISHED;
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
