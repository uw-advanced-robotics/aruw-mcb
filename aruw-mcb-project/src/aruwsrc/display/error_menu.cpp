/*
 * Copyright (c) 2020-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "error_menu.hpp"

#include "tap/drivers.hpp"
#include "tap/errors/error_controller.hpp"
#include "tap/errors/system_error.hpp"

namespace aruwsrc
{
namespace display
{
ErrorMenu::ErrorMenu(
    modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> > *vs,
    tap::Drivers *drivers,
    int entriesToDisplay)
    : AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >(vs, ERROR_MENU_ID),
      drivers(drivers),
      vertScrollHandler(drivers, 0, entriesToDisplay)
{
}

void ErrorMenu::update() {}

void ErrorMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    if (button == modm::MenuButtons::LEFT)
    {
        this->remove();
    }
    else
    {
        vertScrollHandler.onShortButtonPress(button);
    }
}

bool ErrorMenu::hasChanged()
{
    bool cursorChanged = vertScrollHandler.acknowledgeCursorChanged();

    size_t currentErrorCount = drivers->errorController.getErrorListSize();
    bool errorCountChanged = (currentErrorCount != prevErrorCount);

    if (errorCountChanged)
    {
        prevErrorCount = currentErrorCount;
        vertScrollHandler.setSize(currentErrorCount);
    }

    return cursorChanged || errorCountChanged;
}

void ErrorMenu::draw()
{
    modm::GraphicDisplay &display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << ErrorMenu::getMenuName() << modm::endl;

    int numErrors = drivers->errorController.getErrorListSize();
    if (numErrors == 0)
    {
        display << "No Errors" << modm::endl;
        return;
    }
    // return as vertScrollHandler doesn't want a size of 0
    if (numErrors != vertScrollHandler.getSize())
    {
        vertScrollHandler.setSize(numErrors);
    }

    int8_t index = 0;

    // Iterate over the errors
    for (const auto &error : drivers->errorController)
    {
        if (index >= vertScrollHandler.getSmallestIndexDisplayed() &&
            index <= vertScrollHandler.getLargestIndexDisplayed())
        {
            display << (index == vertScrollHandler.getCursorIndex() ? "> " : "  ");

            display << error.getDescription() << modm::endl;
        }
        index++;
    }
}

}  // namespace display
}  // namespace aruwsrc