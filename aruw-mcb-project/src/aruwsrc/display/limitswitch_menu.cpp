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

#include "limitswitch_menu.hpp"
#include "aruwsrc/communication/sensors/beam_break/beam_break.hpp"
#include <algorithm>
#include <cmath>
#include "tap/communication/gpio/digital.hpp"
#include "tap/drivers.hpp"

using namespace aruwsrc::communication::sensors::beam_break;

namespace aruwsrc::display
{

LimitSwitchMenu::LimitSwitchMenu(
    modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> >* stack,
    Drivers* drivers)
    : AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >(stack, LIMITSWITCH_MENU_ID),
      drivers(drivers)
{
} 

void LimitSwitchMenu::drawLimitSwitch(tap::gpio::Digital::InputPin pin)
{
    DigitalBeamBreak beamBreak(&(drivers->digital), pin, false);
    
    getViewStack()->getDisplay() << "Pin " << pin << ": ";
    
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

    

    for (const auto& [pin,status] : pins) {
    
        drawLimitSwitch(pin);
    }

}

void LimitSwitchMenu::update()
{
    if (this->hasChanged())
    {
        this->draw();
    }
}

bool LimitSwitchMenu::hasChanged()
{
    bool statusChanged = false;
    
    for (auto& [pin, status] : pins) {
        
        DigitalBeamBreak beamBreak(&(drivers->digital), pin, false);
        bool currState = beamBreak.getLimitSwitchDepressed();
        
        if (currState != status) {
            status = currState;  
            statusChanged = true;
        }
    }
    
    return statusChanged;
}

void LimitSwitchMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    if (button == modm::MenuButtons::LEFT)
    {
        this->remove();
    }
}
}  // namespace display

