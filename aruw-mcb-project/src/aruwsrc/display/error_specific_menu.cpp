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

#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"
#include "tap/errors/error_controller.hpp"
#include "tap/errors/system_error.hpp"

#include "error_menu.hpp"

namespace aruwsrc::display
{
ErrorSpecificMenu::ErrorSpecificMenu(
    modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> >* vs,
    tap::Drivers* drivers,
    int errorIndex)
    : AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >(vs, 1),
      drivers(drivers),
      index(errorIndex)
{
}

void ErrorSpecificMenu::draw()
{
    const auto& errorList = drivers->errorController.getErrorList();
    const tap::errors::SystemError* currError = nullptr;
    if (index < 0 || index >= static_cast<int>(errorList.getSize()))
    {
        RAISE_ERROR(drivers, "ErrorSpecificMenu has invalid error index");
        return;
    }

    int idx = 0;
    for (const auto& err : errorList)
    {
        if (idx == index)
        {
            currError = &err;
            break;
        }
        idx++;
    }

    if (currError == nullptr)
    {
        RAISE_ERROR(drivers, "ErrorSpecificMenu could not find error");
        return;
    }

    modm::GraphicDisplay& display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);

    currFile = currError->getFilename();
    currDescription = currError->getDescription();
    currLineNum = currError->getLineNumber();

    size_t MAX_CHARS_PER_LINE = display.getWidth() / display.getStringWidth("a");
    std::string wrappedFile = wrapText(currFile, MAX_CHARS_PER_LINE);
    display << wrappedFile.c_str();
    display << modm::endl << modm::endl;

    display << "Line: " << currLineNum << modm::endl;
}

void ErrorSpecificMenu::update() {}

void ErrorSpecificMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    if (button == modm::MenuButtons::LEFT)
    {
        this->remove();
    }
}

bool ErrorSpecificMenu::hasChanged()
{
    const auto& errorList = drivers->errorController.getErrorList();
    int idx = 0;
    for (const auto& err : errorList)
    {
        if (idx == index)
        {
            bool sameDes = (currDescription == err.getDescription());
            bool sameName = (currFile == err.getFilename());
            bool sameLine = (currLineNum == static_cast<int>(err.getLineNumber()));
            return !(sameDes && sameName && sameLine);
        }
        idx++;
    }

    return false;
}

}  // namespace aruwsrc::display