/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "about_menu.hpp"

#include "tap/drivers.hpp"

namespace aruwsrc::display
{
AboutMenu::AboutMenu(modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> > *vs)
    : AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >(vs, TURRET_MCB_MENU_ID)

{
}

void AboutMenu::draw()
{
    modm::GraphicDisplay &display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << getMenuName() << modm::endl;

    display << "Robot Name: " << ROBOT_NAME << modm::endl;
    display << "Last User: " << LAST_USER << modm::endl;
    display << "Sha: " << LAST_SHA << modm::endl;
    display << "Last Built: " << LAST_DATETIME << modm::endl;
    display << "Branch Name: " << BRANCH_NAME << modm::endl;
    drawn = true;
}

void AboutMenu::update() {}

void AboutMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    if (button == modm::MenuButtons::LEFT)
    {
        this->remove();
    }
}
bool AboutMenu::hasChanged() { return !drawn; }

}  // namespace aruwsrc::display
