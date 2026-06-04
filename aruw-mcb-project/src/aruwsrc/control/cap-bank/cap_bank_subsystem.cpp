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

    using Mode = communication::can::cap_bank::Mode;

    // A requested safety discharge takes priority and latches until the bank reports it
    // has finished (returned to STANDBY). While latched we keep commanding the discharge.
    if (this->safetyDischargeRequested)
    {
        if (this->capacitorBank.getMode() == Mode::STANDBY)
        {
            this->safetyDischargeRequested = false;
        }
        else
        {
            this->capacitorBank.setMode(Mode::SAFETY_DISCHARGE);
            return;
        }
    }

    // Otherwise publish the desired mode derived purely from the MCB's own intent. setMode()
    // is idempotent and doubles as the bank's heartbeat, so we send it every tick.
    Mode desired;
    if (!this->capacitorsEnabled)
    {
        this->capacitorBank.setSprinting(communication::can::cap_bank::SprintMode::NO_SPRINT);
        desired = Mode::STANDBY;
    }
    else if (this->capacitorBank.isSprinting())
    {
        desired = Mode::BOOST;
    }
    else
    {
        desired = Mode::CHARGE_ONLY;
    }

    this->capacitorBank.setMode(desired);
}
}  // namespace aruwsrc::control::cap_bank
