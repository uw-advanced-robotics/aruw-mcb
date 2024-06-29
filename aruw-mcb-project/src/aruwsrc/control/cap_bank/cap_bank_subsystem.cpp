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

namespace aruwsrc::control::capbank
{
CapBankSubsystem::CapBankSubsystem(
    tap::Drivers* drivers,
    can::capbank::CapacitorBank& capacitorBank)
    : Subsystem(drivers),
      capacitorBank(capacitorBank),
      capacitorsEnabled(false)
{
    capacitorBank.setSprinting(can::capbank::SprintMode::NO_SPRINT);
    this->messageTimer.restart(20);
}

void CapBankSubsystem::refresh()
{
    if (this->messageTimer.execute())
    {
        messageTimer.restart(20);

        if (!this->enabled() && !this->capacitorBank.isDisabled())
        {
            this->capacitorBank.setSprinting(can::capbank::SprintMode::NO_SPRINT);
            this->capacitorBank.stop();
        }
        else if (this->enabled() && !this->capacitorBank.isEnabled())
        {
            this->capacitorBank.start();
        }
        else
        {
            this->capacitorBank.ping();
        }
    }
}

}  // namespace aruwsrc::control::capbank
