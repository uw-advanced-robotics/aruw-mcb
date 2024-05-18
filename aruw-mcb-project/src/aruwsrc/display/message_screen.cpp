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

#include "message_screen.hpp"

namespace aruwsrc
{
namespace display
{
MessageScreen::MessageScreen(modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView>> *vs, const char* msg)
    : AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >(vs, MESSAGE_SCREEN_ID),
      msg(msg)
{
}

void MessageScreen::update()
{
    if (this->hasChanged())
    {
        this->draw();
    }
}

void MessageScreen::shortButtonPress(modm::MenuButtons::Button button)
{

}

bool MessageScreen::hasChanged()
{
    return true;
}

void MessageScreen::draw()
{
    modm::GraphicDisplay &display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << msg;
    // TODO implement, see issue #222
}
}  // namespace display
}  // namespace aruwsrc
