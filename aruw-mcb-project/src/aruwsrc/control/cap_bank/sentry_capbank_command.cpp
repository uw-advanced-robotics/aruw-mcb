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

#include "sentry_capbank_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/serial/remote.hpp"

#include "aruwsrc/communication/can/capacitor_bank.hpp"

namespace aruwsrc::control::capbank
{
SentryCapBankCommand::SentryCapBankCommand(
    tap::Drivers* drivers,
    aruwsrc::control::capbank::CapBankSubsystem& capBankSubsystem)
    : drivers(drivers),
      capBankSubsystem(capBankSubsystem)

{
    addSubsystemRequirement(&capBankSubsystem);
}

void SentryCapBankCommand::initialize() { capBankSubsystem.enableCapacitors(); }

void SentryCapBankCommand::execute() {}

void SentryCapBankCommand::end(bool) { capBankSubsystem.disableCapacitors(); }

bool SentryCapBankCommand::isFinished() const { return false; }

}  // namespace aruwsrc::control::capbank
