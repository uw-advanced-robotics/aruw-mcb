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

#include "sentry_drive_manual_command.hpp"

#include "tap/communication/serial/remote.hpp"

#include "sentry_drive_subsystem.hpp"

#include "aruwsrc/control/control_operator_interface.hpp"

using tap::control::Subsystem;

namespace aruwsrc::control::sentry::drive
{
SentryDriveManualCommand::SentryDriveManualCommand(
    control::ControlOperatorInterface* controlOperatorInterface,
    SentryDriveSubsystem* subsystem)
    : Command(),
      controlOperatorInterface(controlOperatorInterface),
      subsystemSentryDrive(subsystem)
{
    addSubsystemRequirement(dynamic_cast<Subsystem*>(subsystem));
}

void SentryDriveManualCommand::initialize() {}

void SentryDriveManualCommand::execute()
{
    subsystemSentryDrive->setDesiredRpm(controlOperatorInterface->getSentrySpeedInput());
}

void SentryDriveManualCommand::end(bool) { subsystemSentryDrive->setDesiredRpm(0); }

bool SentryDriveManualCommand::isFinished() const { return false; }
}  // namespace aruwsrc::control::sentry::drive
