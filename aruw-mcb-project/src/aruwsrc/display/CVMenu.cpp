/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include <aruwlib/display/CVMenu.hpp>
#include <aruwlib/display/MenuIdentifiers.h>

#include "aruwlib/Drivers.hpp"
#include "aruwlib/communication/serial/uart.hpp"

#include "main.hpp"

namespace aruwlib
{
namespace display
{
CVMenu::CVMenu(modm::ViewStack *vs, Drivers *drivers)
    : AbstractMenu(vs, CV_MENU_ID),
      drivers(drivers),
      verticalScrollLogic(new VerticalScrollLogicHandler(drivers, 3, 8))
{
}

void CVMenu::update()
{
    if (this->hasChanged())
    {
        this->draw();
    }
}

void CVMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    verticalScrollLogic->onShortButtonPress(button);
    if (!verticalScrollLogic->acknowledgeCursorChanged())
    {
        if (button == modm::MenuButtons::LEFT)
        {
            this->remove();
        }
    }
    else
    {
        if (button == modm::MenuButtons::OK)
        {
            switch (verticalScrollLogic->getCursorIndex())
            {
                case 0:
                    xavierSerial.beginAutoAim();
                    break;

                case 1:
                    xavierSerial.stopAutoAim();
                    break;

                case 3:
                    // TODO
                    break;

                default:
                    break;
            }
        }
    }
}

bool CVMenu::hasChanged()
{
    // TODO implement, see issue #222
    // This should return true only when the state of the CVMenu has changed
    // (the stuff on the display has changed) to minimize I/O usage.
    return true;
}

void CVMenu::draw()
{
    modm::GraphicDisplay &display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << CVMenu::getMenuName();

    display.setCursor(0, 3);
    display << "UART Connection Status: " << ((true) ? "CONNECTED" : "NOT CONNECTED");

    display.setCursor(0, verticalScrollLogic->getCursorIndex() + 4);
    display << ">";

    display.setCursor(1, 4);
    display << "Start Tracking";

    display.setCursor(1, 5);
    display << "Stop Tracking";

    display.setCursor(1, 6);
    display << "Restart Jetson";
}
}  // namespace display
}  // namespace aruwlib
