/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "engineer_manual_drive_command.hpp"

#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"

namespace aruwsrc::engineer::chassis
{
// class HolonomicChassisSubsystem;

EngineerManualDriveCommand::EngineerManualDriveCommand(
    tap::Drivers* drivers,
    EngineerControlOperatorInterface* operatorInterface,
    aruwsrc::chassis::HolonomicChassisSubsystem* chassis)
    : drivers(drivers),
      operatorInterface(operatorInterface),
      chassis(chassis),
      chassisDriveCommand(drivers, operatorInterface, chassis)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void EngineerManualDriveCommand::initialize() { chassisDriveCommand.initialize(); }

void EngineerManualDriveCommand::execute()
{
    if (operatorInterface->isDriveMode())
    {
        chassisDriveCommand.execute();
    }
    else
    {
        chassis->setZeroRPM();
    }
}

void EngineerManualDriveCommand::end(bool) { chassis->setZeroRPM(); }

bool EngineerManualDriveCommand::isFinished() const { return false; }

}  // namespace aruwsrc::engineer::chassis
