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

#include "chassis_drive_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/drivers.hpp"

#include "new-chassis/chassis_subsystem.hpp"

#include "chassis_rel_drive.hpp"

namespace aruwsrc
{
namespace chassis
{
ChassisDriveCommand::ChassisDriveCommand(
    tap::Drivers* drivers,
    aruwsrc::control::ControlOperatorInterface* operatorInterface,
    ChassisSubsystem* chassis)
    : drivers(drivers),
      operatorInterface(operatorInterface),
      chassis(chassis)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisDriveCommand::initialize() {}

void ChassisDriveCommand::execute()
{
    ChassisRelDrive::onExecute(operatorInterface, drivers, chassis);
}

void ChassisDriveCommand::end(bool) { chassis->setZeroRPM(); }

bool ChassisDriveCommand::isFinished() const { return false; }

}  // namespace chassis

}  // namespace aruwsrc
