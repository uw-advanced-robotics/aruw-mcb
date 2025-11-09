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

#ifndef SENTRY_CAPBANK_COMMAND_HPP
#define SENTRY_CAPBANK_COMMAND_HPP

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/cap_bank/cap_bank_subsystem.hpp"

//using namespace aruwsrc::control::sentry;

namespace aruwsrc::sentry::chassis
{

class SentryCapBankCommand : public tap::control::Command
{
public:
    SentryCapBankCommand(
        tap::Drivers* drivers, 
        aruwsrc::control::capbank::CapBankSubsystem& capBankSubsystem);

    void initialize() override;

    
    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "sentry capbank"; }

private:
    tap::Drivers* drivers;
    aruwsrc::control::capbank::CapBankSubsystem& capBankSubsystem;
};  // class SentryCapBankCommand

}  // namespace aruwsrc::sentry::chassis

#endif  // SENTRY_CAPBANK_COMMAND_HPP_
