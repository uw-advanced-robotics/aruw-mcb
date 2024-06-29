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

#include "tap/architecture/timeout.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/communication/can/capacitor_bank.hpp"

namespace tap
{
class Drivers;
}

namespace aruwsrc::control::capbank
{
class CapBankSubsystem : public tap::control::Subsystem
{
public:
    CapBankSubsystem(tap::Drivers* drivers, can::capbank::CapacitorBank& capacitorBank);

    virtual ~CapBankSubsystem() {}
    const char* getName() const override { return "Capacitor Bank"; }

    void changeSprintMode(aruwsrc::can::capbank::SprintMode mode) const { this->capacitorBank.setSprinting(mode); }

    void refreshSafeDisconnect() override
    {
        this->disableCapacitors();
        this->capacitorBank.stop();
    }

    void refresh() override;

    void enableCapacitors() { this->capacitorsEnabled = true; }
    void disableCapacitors() { this->capacitorsEnabled = false; }
    void toggleCapacitors() { this->capacitorsEnabled = !this->capacitorsEnabled; }

    bool enabled() const { return this->capacitorsEnabled; }

private:
    can::capbank::CapacitorBank& capacitorBank;

    bool capacitorsEnabled;

    tap::arch::MilliTimeout messageTimer;
};
}  // namespace aruwsrc::control::capbank

#endif  // CAPACITOR_BANK_SUBSYSTEM_HPP_
