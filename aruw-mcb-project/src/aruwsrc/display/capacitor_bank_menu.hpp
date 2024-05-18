/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef CAPACITOR_BANK_MENU_HPP_
#define CAPACITOR_BANK_MENU_HPP_

#include "tap/architecture/periodic_timer.hpp"
#include "tap/display/dummy_allocator.hpp"

#include "aruwsrc/communication/can/capacitor_bank.hpp"
#include "modm/ui/menu/abstract_menu.hpp"

namespace aruwsrc
{
class Drivers;
}  // namespace aruwsrc

namespace aruwsrc::display
{
/**
 * Menu that allows user to see information about the current state of the capacitor bank.
 */
class CapacitorBankMenu
    : public modm::AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >
{
public:
    CapacitorBankMenu(
        modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> >* vs,
        communication::can::capbank::CapacitorBank* capacitorBank);
    void draw() override;

    void update() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    bool hasChanged() override;

    static const char* getMenuName() { return "Capacitor Bank Menu"; }

private:
    static constexpr int TURRET_MCB_MENU_ID = 13;

    communication::can::capbank::CapacitorBank* capacitorBank;

    int milliVolts = 0, milliAmps = 0, powerLimit = 0, availableEnergy = 0;
    communication::can::capbank::State state;

    bool changed;
};
}  // namespace aruwsrc::display

#endif  // CAPACITOR_BANK_MENU_HPP_