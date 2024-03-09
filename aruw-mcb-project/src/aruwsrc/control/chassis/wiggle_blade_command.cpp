/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "wiggle_blade_command.hpp"

namespace aruwsrc::chassis
{

WiggleBladeCommand::WiggleBladeCommand(
    tap::Drivers* drivers,
    HolonomicChassisSubsystem* chassis,
    const aruwsrc::control::turret::TurretMotor* yawMotor,
    aruwsrc::control::ControlOperatorInterface& operatorInterface)
    : tap::control::ComprisedCommand(drivers),
      beybladeCommand(drivers, chassis, yawMotor, operatorInterface),
      wiggleDriveCommand(drivers, chassis, yawMotor, operatorInterface)
{
    comprisedCommandScheduler.registerSubsystem(
        reinterpret_cast<tap::control::Subsystem*>(chassis));
    addSubsystemRequirement(reinterpret_cast<tap::control::Subsystem*>(chassis));
}

bool WiggleBladeCommand::isReady()
{
    return beybladeCommand.isReady() && wiggleDriveCommand.isReady();
}

void WiggleBladeCommand::initialize()
{
    comprisedCommandScheduler.addCommand(&wiggleDriveCommand);
    wiggleTimer.restart(rand() % (MAX_WIGGLE_TIME - MIN_WIGGLE_TIME) + MIN_WIGGLE_TIME);
}

void WiggleBladeCommand::execute()
{
    // done wiggling
    if (wiggleTimer.execute())
    {
        comprisedCommandScheduler.removeCommand(&wiggleDriveCommand, true);
        comprisedCommandScheduler.addCommand(&beybladeCommand);
        beybladeTimer.restart(rand() % (MAX_BEYBLADE_TIME - MIN_BEYBLADE_TIME) + MIN_BEYBLADE_TIME);
    }
    else if (beybladeTimer.execute())
    {
        // done beyblading
        comprisedCommandScheduler.removeCommand(&beybladeCommand, true);
        comprisedCommandScheduler.addCommand(&wiggleDriveCommand);
        wiggleTimer.restart(rand() % (MAX_WIGGLE_TIME - MIN_WIGGLE_TIME) + MIN_WIGGLE_TIME);
    }
    comprisedCommandScheduler.run();
}

void WiggleBladeCommand::end(bool interrupted)
{
    comprisedCommandScheduler.removeCommand(&beybladeCommand, interrupted);
    comprisedCommandScheduler.removeCommand(&wiggleDriveCommand, interrupted);
}

bool WiggleBladeCommand::isFinished() const { return false; }

}  // namespace aruwsrc::chassis
