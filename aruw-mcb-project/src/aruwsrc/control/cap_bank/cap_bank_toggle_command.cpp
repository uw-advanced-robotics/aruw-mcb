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

#include "cap_bank_toggle_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/serial/remote.hpp"

#include "aruwsrc/communication/can/capacitor_bank.hpp"

namespace aruwsrc::control::cap_bank
{

CapBankToggleCommand::CapBankToggleCommand(
    tap::Drivers* drivers,
    aruwsrc::control::cap_bank::CapBankSubsystem& capBankSubsystem)
    : drivers(drivers),
      capBankSubsystem(capBankSubsystem)

{
    addSubsystemRequirement(&capBankSubsystem);
}

void CapBankToggleCommand::initialize() { capBankSubsystem.toggleCapacitors(); }

void CapBankToggleCommand::execute() {}

void CapBankToggleCommand::end(bool) {}

bool CapBankToggleCommand::isFinished() const { return true; }

}  // namespace aruwsrc::control::cap_bank
