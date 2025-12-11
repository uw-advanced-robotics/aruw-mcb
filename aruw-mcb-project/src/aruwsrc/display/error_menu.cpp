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
    switch (button)
    {
        case modm::MenuButtons::LEFT:
            this->remove();
            break;
        case modm::MenuButtons::UP:
        case modm::MenuButtons::DOWN:
            vertScrollHandler.onShortButtonPress(button);
            rightTapNum = 0;
            break;
        case modm::MenuButtons::RIGHT:
            rightTapNum++;
            if (rightTapNum < 2)
            {
                break;
            }
            drivers->errorController.removeSystemErrorAtIndex(vertScrollHandler.getCursorIndex());
            rightTapNum = 0;
            break;
        case modm::MenuButtons::OK:
            break;
        default:
            break;
    }
}

bool ErrorMenu::hasChanged()
{
    bool cursorChanged = vertScrollHandler.acknowledgeCursorChanged();

    size_t currentErrorCount = drivers->errorController.getErrorList().getSize();
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
    display << "Tap RIGHT twice to remove error" << modm::endl;

    int numErrors = drivers->errorController.getErrorList().getSize();
    if (numErrors == 0)
    {
        display << "No Errors" << modm::endl;
        return;
    }

    if (numErrors != vertScrollHandler.getSize())
    {
        vertScrollHandler.setSize(numErrors);
    }

    int8_t index = 0;

    // There is no way to get the font width directly, but you can give it a character and get its
    // width. There is also no getFont(), so we cannot see the font, only works with monospaced
    // fonts.
    size_t MAX_CHARS_PER_LINE = display.getWidth() / display.getStringWidth("a");

    for (const auto &error : drivers->errorController.getErrorList())
    {
        if (index >= vertScrollHandler.getSmallestIndexDisplayed() &&
            index <= vertScrollHandler.getLargestIndexDisplayed())
        {
            // Draw selector
            bool isSelected = (index == vertScrollHandler.getCursorIndex());
            display << (isSelected ? "> " : "  ");

            const std::string text = std::string(error.getDescription()) + " [" +
                                     std::string(error.getFilename()) + ':' +
                                     std::to_string(error.getLineNumber()) + ']';

            size_t currentLineLen = 2;  // Start at 2 because of "> " or "  "
            size_t pos = 0;
            // Print error with wrapping
            while (pos < text.size())
            {
                // manual scan for next space
                size_t nextSpace = pos;
                while (nextSpace < text.size() && text[nextSpace] != ' ') nextSpace++;

                size_t wordLen = nextSpace - pos;
                bool needsSpace = (currentLineLen > 2);

                // wrap check
                if (currentLineLen + wordLen + (needsSpace ? 1 : 0) > MAX_CHARS_PER_LINE)
                {
                    display << modm::endl << "  ";
                    currentLineLen = 2;
                    needsSpace = false;
                }

                if (needsSpace)
                {
                    display << ' ';
                    currentLineLen++;
                }

                // print characters
                for (size_t i = pos; i < pos + wordLen; ++i) display << text[i];

                currentLineLen += wordLen;
                pos = (nextSpace < text.size()) ? nextSpace + 1 : nextSpace;
            }

            // End the error item
            display << modm::endl;
        }
        // move to next error
        index++;
    }
}

}  // namespace display
}  // namespace aruwsrc