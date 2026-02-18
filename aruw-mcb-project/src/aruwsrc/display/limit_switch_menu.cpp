/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "limit_switch_menu.hpp"

#include <algorithm>
#include <cmath>

#include "tap/communication/gpio/digital.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/communication/sensors/beam_break/beam_break.hpp"
#include "aruwsrc/drivers_singleton.hpp"

using namespace aruwsrc::communication::sensors::beam_break;

namespace aruwsrc::display
{
LimitSwitchMenu::LimitSwitchMenu(
    modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> >* stack,
    tap::Drivers* drivers)
    : AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >(stack, LIMIT_SWITCH_MENU_ID),
      drivers(drivers)
{
}

void LimitSwitchMenu::drawLimitSwitch(Digital::InputPin pin)
{
    DigitalBeamBreak beamBreak(&drivers->digital, pin, false);
    const char* pinName = "";

    std::size_t idx = static_cast<std::size_t>(pin);
    if (idx < InputPinNames.size())
    {
        pinName = InputPinNames[idx].data();
    }
    else
    {
        pinName = "UNKNOWN PIN";
    }

    getViewStack()->getDisplay() << "Pin " << pinName << ": ";

    if (beamBreak.getLimitSwitchDepressed())
    {
        getViewStack()->getDisplay() << "1";
        pins[pin] = 1;
    }
    else
    {
        getViewStack()->getDisplay() << "0";
        pins[pin] = 0;
    }

    getViewStack()->getDisplay() << modm::endl;
}

void LimitSwitchMenu::draw()
{
    modm::GraphicDisplay& display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << getMenuName() << modm::endl;

    for (const auto& [pin, status] : pins)
    {
        drawLimitSwitch(pin);
    }
}

void LimitSwitchMenu::update() {}

bool LimitSwitchMenu::hasChanged()
{
    for (auto& [pin, status] : pins)
    {
        DigitalBeamBreak beamBreak(&(drivers->digital), pin, false);
        int currState = beamBreak.getLimitSwitchDepressed();
        if (currState != status)
        {
            return true;
        }
    }

    return false;
}

void LimitSwitchMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    if (button == modm::MenuButtons::LEFT)
    {
        this->remove();
    }
}

// void LimitSwitchMenu::setPinValue(Digital::InputPin pin, int val) {
//     for (const auto& pair : pins) {
//         if (pair.first == pin) {
//             pair.second = val;           wait how can i modify smth thats static constexpr
//         }
//     }
// }

}  // namespace aruwsrc::display
