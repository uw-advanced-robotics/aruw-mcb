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
    communication::can::capbank::CapacitorBank *capacitorBank)
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
    display << "State: ";

    switch (this->state)
    {
        case communication::can::capbank::State::RESET:
            display << "RESET";
            break;
        case communication::can::capbank::State::SAFE:
            display << "SAFE";
            break;
        case communication::can::capbank::State::CHARGE:
            display << "CHARGE";
            break;
        case communication::can::capbank::State::CHARGE_DISCHARGE:
            display << "CHARGE_DISCHARGE";
            break;
        case communication::can::capbank::State::DISCHARGE:
            display << "DISCHARGE";
            break;
        case communication::can::capbank::State::BATTERY_OFF:
            display << "BATTERY_OFF";
            break;
        case communication::can::capbank::State::FAILURE:
            display << "FAILURE";
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
        this->state != this->capacitorBank->getState())
    {
        this->milliAmps = this->capacitorBank->getCurrent() * 1000;
        this->milliVolts = this->capacitorBank->getVoltage() * 1000;
        this->powerLimit = this->capacitorBank->getPowerLimit();
        this->availableEnergy = this->capacitorBank->getAvailableEnergy();
        this->state = this->capacitorBank->getState();
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

bool CapacitorBankMenu::hasChanged() { return changed; }

}  // namespace aruwsrc::display