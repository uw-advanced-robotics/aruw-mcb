/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "cv_menu.hpp"

#include "tap/drivers.hpp"

#include "aruwsrc/drivers.hpp"

namespace aruwsrc::display
{
CVMenu::CVMenu(
    modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> > *vs,
    aruwsrc::Drivers *drivers)
    : AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >(vs, CV_MENU_ID),
      drivers(drivers),
      verticalScroll(
          drivers,
          MODM_ARRAY_SIZE(CV_MENU_UPDATE_FNS),
          std::min(DISPLAY_MAX_ENTRIES, static_cast<uint8_t>(MODM_ARRAY_SIZE(CV_MENU_UPDATE_FNS))))
{
}

void CVMenu::draw()
{
    modm::GraphicDisplay &display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << getMenuName() << modm::endl;

    for (int8_t i = verticalScroll.getSmallestIndexDisplayed();
         i <= verticalScroll.getLargestIndexDisplayed();
         i++)
    {
        display << (i == verticalScroll.getCursorIndex() ? "> " : "  ");
        (this->*std::get<0>(CV_MENU_UPDATE_FNS[i]))(display);
        display << modm::endl;
    }
}

void CVMenu::update() {}

void CVMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    if (button == modm::MenuButtons::LEFT)
    {
        this->remove();
    }
    else if (button == modm::MenuButtons::RIGHT)
    {
        (this->*std::get<1>(CV_MENU_UPDATE_FNS[verticalScroll.getCursorIndex()]))();
    }
    else
    {
        verticalScroll.onShortButtonPress(button);
    }
}

bool CVMenu::hasChanged()
{
    return verticalScroll.acknowledgeCursorChanged() || updatePeriodicTimer.execute();
}

void CVMenu::drawShutdownCV(modm::IOStream &stream) { stream << "Shutdown CV"; }

void CVMenu::shutdownCV() { drivers->visionCoprocessor.sendShutdownMessage(); }

void CVMenu::drawRebootCV(modm::IOStream &stream) { stream << "Reboot CV"; }

void CVMenu::rebootCV() { drivers->visionCoprocessor.sendRebootMessage(); }

void CVMenu::drawCVOnline(modm::IOStream &stream)
{
    stream << "CV Online: " << drivers->visionCoprocessor.isCvOnline();
}

}  // namespace aruwsrc::display
