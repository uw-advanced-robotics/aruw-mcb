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

#include "capacitor_bank_menu.hpp"

#include "tap/drivers.hpp"

namespace aruwsrc::display
{
CapacitorBankMenu::CapacitorBankMenu(
    modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> > *vs,
    communication::can::cap_bank::CapacitorBank *capacitorBank)
    : AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >(vs, TURRET_MCB_MENU_ID),
      capacitorBank(capacitorBank)
{
}

void CapacitorBankMenu::draw()
{
    modm::GraphicDisplay &display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << getMenuName() << modm::endl;

    display << "Cap Voltage: " << milliVolts << "mV" << modm::endl;
    display << "Output Current: " << milliAmps << "mA" << modm::endl;
    display << "Max Charge Speed: " << powerLimit << "W" << modm::endl;
    display << "Available Energy: " << availableEnergy << "J" << modm::endl;
    display << "Mode: ";

    switch (this->mode)
    {
        case communication::can::cap_bank::Mode::STANDBY:
            display << "STANDBY";
            break;
        case communication::can::cap_bank::Mode::CHARGE_ONLY:
            display << "CHARGE_ONLY";
            break;
        case communication::can::cap_bank::Mode::BOOST:
            display << "BOOST";
            break;
        case communication::can::cap_bank::Mode::SAFETY_DISCHARGE:
            display << "SAFETY_DISCHARGE";
            break;
        default:
            display << "UNKNOWN";
            break;
    }
    display << modm::endl;
}

void CapacitorBankMenu::update()
{
    if (this->milliAmps != this->capacitorBank->getCurrent() * 1000 ||
        this->milliVolts != this->capacitorBank->getVoltage() * 1000 ||
        this->mode != this->capacitorBank->getMode())
    {
        this->milliAmps = this->capacitorBank->getCurrent() * 1000;
        this->milliVolts = this->capacitorBank->getVoltage() * 1000;
        this->powerLimit = this->capacitorBank->getPowerLimit();
        this->availableEnergy = this->capacitorBank->getAvailableEnergy();
        this->mode = this->capacitorBank->getMode();
        this->changed = true;
    }
}

void CapacitorBankMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    if (button == modm::MenuButtons::LEFT)
    {
        this->remove();
    }
}

bool CapacitorBankMenu::hasChanged() { return changed && updateTimer.execute(); }

}  // namespace aruwsrc::display