/*
 * Copyright (c) 2021-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SENTRY_CAP_BANK_COMMAND_HPP_
#define SENTRY_CAP_BANK_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/cap-bank/cap_bank_subsystem.hpp"

namespace aruwsrc::control::capbank
{
class SentryCapBankCommand : public tap::control::Command
{
public:
    SentryCapBankCommand(
        tap::Drivers* drivers,
        aruwsrc::control::cap_bank::CapBankSubsystem& capBankSubsystem);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "Sentry Cap Bank"; }

private:
    tap::Drivers* drivers;
    aruwsrc::control::cap_bank::CapBankSubsystem& capBankSubsystem;
};  // class SentryCapBankCommand

}  // namespace aruwsrc::control::capbank

#endif  // SENTRY_CAPBANK_COMMAND_HPP_
