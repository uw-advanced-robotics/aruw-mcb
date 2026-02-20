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

#include "error_specific_menu.hpp"
#include "error_menu.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/error_controller.hpp"
#include "tap/errors/system_error.hpp"
#include "tap/errors/create_errors.hpp"

namespace aruwsrc::display {
ErrorSpecificMenu::ErrorSpecificMenu(
    modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> > *vs,
    tap::Drivers *drivers,
    const tap::errors::SystemError* error)
    : AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >(vs, 1),
      drivers(drivers),
      currError(error)
{}

void ErrorSpecificMenu::draw() {
    if (currError == nullptr)
    {
        RAISE_ERROR(drivers, "ErrorSpecificMenu has a nullptr error");
        return;
    }

    modm::GraphicDisplay &display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);

    currFile = currError->getFilename();
    currDescription = currError->getDescription();
    currLineNum = currError->getLineNumber();

       
    size_t MAX_CHARS_PER_LINE = display.getWidth() / display.getStringWidth("a");
    std::string linenum = std::to_string(currLineNum) + ": ";
    std::string wrapped = wrapText(currDescription, MAX_CHARS_PER_LINE - linenum.length());

    display << linenum.c_str();
    for (char c : wrapped) {
        display << c;
    }
    display << modm::endl;
}

void ErrorSpecificMenu::update() {}

void ErrorSpecificMenu::shortButtonPress(modm::MenuButtons::Button button) {
    if (button == modm::MenuButtons::LEFT)
    {
        this->remove();
    } 
}

bool ErrorSpecificMenu::hasChanged() {
    bool sameDes = (currDescription == currError->getDescription());
    bool sameName = (currFile == currError->getFilename());
    bool sameLine = (currLineNum == currError->getLineNumber());
    return !(sameDes && sameName && sameLine);
}

}