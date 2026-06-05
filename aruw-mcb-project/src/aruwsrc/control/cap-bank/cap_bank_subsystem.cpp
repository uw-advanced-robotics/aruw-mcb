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

#include "cap_bank_subsystem.hpp"

namespace aruwsrc::control::cap_bank
{
CapBankSubsystem::CapBankSubsystem(
    tap::Drivers* drivers,
    communication::can::cap_bank::CapacitorBank& capacitorBank)
    : Subsystem(drivers),
      capacitorBank(capacitorBank),
      capacitorsEnabled(false),
      capBankTestCommand(this)
{
    this->setTestCommand(&this->capBankTestCommand);
    this->capacitorBank.setSprinting(communication::can::cap_bank::SprintMode::NO_SPRINT);
    this->messageTimer.restart(20);
}

void CapBankSubsystem::refresh()
{
    if (!this->messageTimer.execute())
    {
        return;
    }
    messageTimer.restart(20);

    using communication::can::cap_bank::CapCommandMode;

    // The MCB only relays intent: a mode (from enable/sprint) plus, inside sendCascadeCommand(), the
    // measured bus current/voltage and the referee power limit. The cap firmware does the control.
    CapCommandMode mode;
    if (!this->capacitorsEnabled)
    {
        this->capacitorBank.setSprinting(communication::can::cap_bank::SprintMode::NO_SPRINT);
        mode = CapCommandMode::OFF;
    }
    else if (this->capacitorBank.isSprinting())
    {
        mode = CapCommandMode::DISCHARGE;
    }
    else
    {
        mode = CapCommandMode::CHARGE;
    }

    this->capacitorBank.sendCascadeCommand(mode);
}
}  // namespace aruwsrc::control::cap_bank
