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

#ifndef CAP_BANK_SUBSYSTEM_HPP_
#define CAP_BANK_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/communication/can/capacitor_bank.hpp"

namespace tap
{
class Drivers;
}

namespace aruwsrc::control::cap_bank
{

enum SprintMode
{
    REGULAR = 0,
    SPRINT = 1
};

class CapBankSubsystem : public tap::control::Subsystem
{
public:
    CapBankSubsystem(
        tap::Drivers* drivers,
        aruwsrc::communication::can::capbank::CapacitorBank& capacitorBank);

    virtual ~CapBankSubsystem() {}
    const char* getName() const override { return "Capacitor Bank"; }

    void toggleCapacitors() { this->enabled = !this->enabled; }
    void toggleDischarge() { this->enabled = false; }

    void changeSprintMode(SprintMode mode);

    void refreshSafeDisconnect() override
    {
        this->enabled = false;
    }

    void refresh() override;

private:
    aruwsrc::communication::can::capbank::CapacitorBank& capacitorBank;

    bool enabled;

    const float SPRINT_MODIFIER = 1.0f;
    const float REGULAR_MODIFIER = 0.2f;
    const float BASE_MODIFIER = 0.4f;

    float desiredSprintModifier = REGULAR_MODIFIER;

    int8_t messageTimer = 0;

    const float SPRINT_SCALE_CHUNK_SIZE = 0.01f;

    float debug = 0;
};
}  // namespace aruwsrc::control::cap_bank

#endif  // CAPACITOR_BANK_SUBSYSTEM_HPP_