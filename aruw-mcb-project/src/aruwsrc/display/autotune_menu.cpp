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

#include "autotune_menu.hpp"

#include "tap/drivers.hpp"

#include "aruwsrc/control/autotune/gravity_autotune.hpp"

// weak function defined if not specifed by user
modm_weak aruwsrc::control::autotune::GravityAutotuneBase** getGravityAutotuneCommands()
{
    return nullptr;
}
namespace aruwsrc::display
{
AutotuneMenu::AutotuneMenu(
    modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> >* vs,
    tap::Drivers* drivers,
    int entriesToDisplay)
    : AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >(vs, 1),
      drivers(drivers),
      verticalScroll(drivers, 0, entriesToDisplay)
{
    verticalScroll.setSize(getCommandNumber());
}

void AutotuneMenu::draw()
{
    modm::GraphicDisplay& display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << getMenuName() << modm::endl;
    if (getCommandNumber() == 0)
    {
        display << "No autotune commands";
        return;
    }

    auto commandMinIndex = verticalScroll.getSmallestIndexDisplayed();
    auto commandMaxIndex = std::min(
        getCommandNumber() - 1,
        static_cast<int>(verticalScroll.getLargestIndexDisplayed()));

    for (int8_t commandId = commandMinIndex; commandId <= commandMaxIndex; ++commandId)
    {
        display << (verticalScroll.getCursorIndex() == commandId ? "> " : "  ");
        display << "Gravity Autotune Command " << (commandId + 1) << modm::endl;
    }
}

void AutotuneMenu::update() {}

bool AutotuneMenu::hasChanged() { return verticalScroll.acknowledgeCursorChanged(); }

void AutotuneMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    switch (button)
    {
        case modm::MenuButtons::LEFT:
            this->remove();
            break;
        case modm::MenuButtons::RIGHT:
        {
            // If there are no commands, do nothing.
            if (getCommandNumber() == 0)
            {
                break;
            }

            int8_t idx = verticalScroll.getCursorIndex();
            // Index is selecting a gravity autotune command, so push the corresponding menu.
            if (idx < getGravityAutotuneCommandNumber())
            {
                this->getViewStack()->push(new GravityAutotuneMenu(
                    getViewStack(),
                    drivers,
                    getGravityAutotuneCommands()[idx]));
            }
            break;
        }
        case modm::MenuButtons::DOWN:
            verticalScroll.onShortButtonPress(modm::MenuButtons::DOWN);
            break;
        case modm::MenuButtons::UP:
            verticalScroll.onShortButtonPress(modm::MenuButtons::UP);
            break;
        case modm::MenuButtons::OK:
            break;
    }
}

}  // namespace aruwsrc::display
