/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "HardwareTestMenu.hpp"
#include "aruwlib/control/command_scheduler.hpp"
#include "aruwlib/control/subsystem.hpp"

namespace aruwlib
{
namespace display
{
HardwareTestMenu::HardwareTestMenu(modm::ViewStack *vs, Drivers *drivers) : AbstractMenu(vs, 2), drivers(drivers)
{
}

void HardwareTestMenu::update()
{
    if (this->hasChanged())
    {
        this->draw();
    }
}

void HardwareTestMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    switch (button)
    {
        case modm::MenuButtons::LEFT:
            this->getViewStack()->pop();
            break;
        case modm::MenuButtons::RIGHT:
            break;
        case modm::MenuButtons::DOWN:
            break;
        case modm::MenuButtons::UP:
            break;
        case modm::MenuButtons::OK:
            break;
    }
}

bool HardwareTestMenu::hasChanged()
{
    // TODO implement, see issue #222
    // This should return true only when the state of the HardwareTestMenu has changed
    // (the stuff on the display has changed) to minimize I/O usage.
    return true;
}

void HardwareTestMenu::draw()
{
    modm::GraphicDisplay &display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << HardwareTestMenu::getMenuName();
    int cursorPosY = 3;
    for(auto& subsystemToCommand : drivers->commandScheduler.getSubsystemToCommandMap()) {
        aruwlib::control::Subsystem* subsystem = subsystemToCommand.first;
        display.setCursor(0, cursorPosY);
        display.setColor(subsystem->isHardwareTestComplete() ? modm::glcd::Color::green() : modm::glcd::Color::red());
        display.fillRectangle(0,cursorPosY, display.getWidth(), display.getHeight());
        display.setColor(modm::glcd::Color::black());
        display << subsystem->getName();
    }
}
}  // namespace display
}  // namespace aruwlib
