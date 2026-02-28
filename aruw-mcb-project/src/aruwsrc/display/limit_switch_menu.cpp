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
    for (size_t i = 0; i < NUM_PINS; ++i)
    {
        pins[i] = {static_cast<tap::gpio::Digital::InputPin>(i), InputPinNames[i], -1};
    }
}

void LimitSwitchMenu::drawLimitSwitch(PinEntry& entry)
{
    DigitalBeamBreak beamBreak(&drivers->digital, entry.pin, false);
    int state = beamBreak.getLimitSwitchDepressed() ? 1 : 0;

    getViewStack()->getDisplay() << "Pin " << entry.name << ": " << state << modm::endl;

    entry.lastState = state;
}

void LimitSwitchMenu::draw()
{
    modm::GraphicDisplay& display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << getMenuName() << modm::endl;

    for (auto& entry : pins)
    {
        drawLimitSwitch(entry);
    }
}

void LimitSwitchMenu::update() {}

bool LimitSwitchMenu::hasChanged()
{
    for (auto& entry : pins)
    {
        DigitalBeamBreak beamBreak(&(drivers->digital), entry.pin, false);
        int currState = beamBreak.getLimitSwitchDepressed();
        if (currState != entry.lastState)
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

}  // namespace aruwsrc::display
