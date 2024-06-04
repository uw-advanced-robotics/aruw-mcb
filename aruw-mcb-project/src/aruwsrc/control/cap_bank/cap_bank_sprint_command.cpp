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

#include "cap_bank_sprint_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/serial/remote.hpp"

namespace aruwsrc::control::cap_bank
{
CapBankSprintCommand::CapBankSprintCommand(
    tap::Drivers* drivers,
    aruwsrc::control::cap_bank::CapBankSubsystem& capBankSubsystem,
    const aruwsrc::can::capbank::SprintMode sprintOption)
    : sprintOption(sprintOption),
      drivers(drivers),
      capBankSubsystem(capBankSubsystem)
{
    addSubsystemRequirement(&capBankSubsystem);
}

void CapBankSprintCommand::initialize() { capBankSubsystem.changeSprintMode(this->sprintOption); }

void CapBankSprintCommand::execute() {}

void CapBankSprintCommand::end(bool)
{
    capBankSubsystem.changeSprintMode(can::capbank::SprintMode::REGULAR);
}

bool CapBankSprintCommand::isFinished() const { return false; }

}  // namespace aruwsrc::control::cap_bank
