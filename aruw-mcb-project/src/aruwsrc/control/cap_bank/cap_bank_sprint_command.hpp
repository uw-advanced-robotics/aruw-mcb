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

#ifndef CAP_BANK_SPRINT_COMMAND_HPP_
#define CAP_BANK_SPRINT_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "aruwsrc/control/cap_bank/cap_bank_subsystem.hpp"
#include "aruwsrc/robot/control_operator_interface.hpp"

namespace aruwsrc::control::cap_bank
{
class CapBankSprintCommand : public tap::control::Command
{
public:
    CapBankSprintCommand(
        tap::Drivers* drivers,
        aruwsrc::control::cap_bank::CapBankSubsystem& capBankSubsystem,
        const aruwsrc::can::capbank::SprintMode sprintOption);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "Cap Bank Sprint"; }

private:
    const aruwsrc::can::capbank::SprintMode sprintOption;

    tap::Drivers* drivers;
    aruwsrc::control::cap_bank::CapBankSubsystem& capBankSubsystem;
};  // class CapBankToggleCommand

}  // namespace aruwsrc::control::cap_bank

#endif  // CAP_BANK_SPRINT_COMMAND_HPP_
