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
modm_weak aruwsrc::control::autotune::GravityAutotuneBase **getGravityAutotuneCommand()
{
    return nullptr;
}
namespace aruwsrc::display
{
AutotuneMenu::AutotuneMenu(
    modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView>> *vs,
    tap::Drivers *drivers)
    : modm::StandardMenu<tap::display::DummyAllocator<modm::IAbstractView>>(vs, AUTOTUNE_MENU_ID),
      drivers(drivers)
{
    // Currently needs to have unfortunate boilerplate to add a higher amount of submenus, 
    // supports two gravity autotune commands, to add more, just copy and paste the pattern below
    for (auto cmdPtr = getGravityAutotuneCommand(); cmdPtr && *cmdPtr; ++cmdPtr)
    {
        auto &menu = gravityMenus.emplace_back(vs, drivers, *cmdPtr);
        const std::size_t idx = gravityMenus.size() - 1;

        switch (idx)
        {
            case 0:
                addEntry(
                    menu.getMenuName(),
                    modm::MenuEntryCallback<tap::display::DummyAllocator<modm::IAbstractView>>(
                        this,
                        &AutotuneMenu::openGravityAutotuneMenu0));
                break;
            case 1:
                addEntry(
                    menu.getMenuName(),
                    modm::MenuEntryCallback<tap::display::DummyAllocator<modm::IAbstractView>>(
                        this,
                        &AutotuneMenu::openGravityAutotuneMenu1));
                break;
        }
    }
}

void AutotuneMenu::initialize() { setTitle(getMenuName()); }

void AutotuneMenu::openSubMenu(size_t index) { getViewStack()->push(&gravityMenus[index]); }

void AutotuneMenu::openGravityAutotuneMenu0() { openSubMenu(0); }
void AutotuneMenu::openGravityAutotuneMenu1() { openSubMenu(1); }

}  // namespace aruwsrc::display
