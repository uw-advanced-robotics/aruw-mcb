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

#include "aruwsrc/control/chassis/new_sentry/sentry_manual_drive_command.hpp"

#include "aruwsrc/robot/sentry/sentry_chassis_rel_drive.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_control_operator_interface.hpp"

using namespace aruwsrc::control::sentry;

namespace aruwsrc
{
namespace control::sentry
{
// class HolonomicChassisSubsystem;

SentryManualDriveCommand::SentryManualDriveCommand(
    tap::Drivers* drivers,
    SentryControlOperatorInterface* operatorInterface,
    chassis::HolonomicChassisSubsystem* chassis)
    : drivers(drivers),
      operatorInterface(operatorInterface),
      chassis(chassis)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void SentryManualDriveCommand::initialize() {}

void SentryManualDriveCommand::execute()
{
    aruwsrc::sentry::SentryChassisRelDrive::onExecute(operatorInterface, drivers, chassis);
}

void SentryManualDriveCommand::end(bool) { chassis->setZeroRPM(); }

bool SentryManualDriveCommand::isFinished() const { return false; }

}  // namespace control::sentry

}  // namespace aruwsrc
