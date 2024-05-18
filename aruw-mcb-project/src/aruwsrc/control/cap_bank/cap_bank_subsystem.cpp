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
    aruwsrc::communication::can::capbank::CapacitorBank& capacitorBank)
    : Subsystem(drivers),
      capacitorBank(capacitorBank)
{
    capacitorBank.setSprintModifier(REGULAR_MODIFIER);
}

void CapBankSubsystem::changeSprintMode(SprintMode mode)
{
    switch (mode)
    {
        case SprintMode::REGULAR:
        {
            desiredSprintModifier =
                ((((float)capacitorBank.getPowerLimit()) / 120) * REGULAR_MODIFIER) + BASE_MODIFIER;
            break;
        }

        case SprintMode::SPRINT:
            desiredSprintModifier = SPRINT_MODIFIER;
            break;

        default:
            break;
    }
}

void CapBankSubsystem::refresh()
{
    if (messageTimer > 20)
    {
        messageTimer = 0;

        if (!this->enabled && !this->capacitorBank.isDisabled())
        {
            this->capacitorBank.stop();
        } 
        else if (this->enabled && !this->capacitorBank.isEnabled())
        {
            this->capacitorBank.start();
        }
    }

    messageTimer++;

    float sprintModifer = capacitorBank.getSprintModifer();

    if (sprintModifer > desiredSprintModifier)
        sprintModifer = SPRINT_SCALE_CHUNK_SIZE;
    else if (sprintModifer < desiredSprintModifier)
        sprintModifer += SPRINT_SCALE_CHUNK_SIZE;

    this->capacitorBank.setSprintModifier(sprintModifer);
}

}  // namespace aruwsrc::control::cap_bank
