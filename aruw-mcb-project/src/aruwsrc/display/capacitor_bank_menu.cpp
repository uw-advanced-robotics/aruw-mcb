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
    communication::sensors::power::ExternalCapacitorBank *capacitorBank)
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

    display << "Voltage: " << milliVolts << "mV" << modm::endl;
    display << "Current: " << milliAmps << "mA" << modm::endl;
    display << "Current Charge Speed: " << milliAmps * milliVolts / 1000 << "mW" << modm::endl;
    display << "Max Charge Speed: " << powerLimit << "W" << modm::endl;
    display << "Available Energy: " << availableEnergy << "J" << modm::endl;
    display << "Status: ";

    switch (this->status)
    {
        case communication::sensors::power::Status::RESET:
            display << "RESET";
            break;
        case communication::sensors::power::Status::CHARGE:
            display << "CHARGE";
            break;
        case communication::sensors::power::Status::CHARGE_DISCHARGE:
            display << "CHARGE_DISCHARGE";
            break;
        case communication::sensors::power::Status::DISCHARGE:
            display << "DISCHARGE";
            break;
        case communication::sensors::power::Status::FAULT:
            display << "FAULT";
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
        this->status != this->capacitorBank->getStatus())
    {
        this->milliAmps = this->capacitorBank->getCurrent() * 1000;
        this->milliVolts = this->capacitorBank->getVoltage() * 1000;
        this->powerLimit = this->capacitorBank->getPowerLimit();
        this->availableEnergy = this->capacitorBank->getAvailableEnergy();
        this->status = this->capacitorBank->getStatus();
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
