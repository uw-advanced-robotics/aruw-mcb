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

#ifndef AUTOTUNE_MENU_HPP_
#define AUTOTUNE_MENU_HPP_

#include <modm/io/iostream.hpp>
#include <modm/ui/menu/menu_entry_callback.hpp>

#include "tap/display/dummy_allocator.hpp"
#include "tap/display/vertical_scroll_logic_handler.hpp"

#include "aruwsrc/control/autotune/gravity_autotune.hpp"
#include "modm/ui/menu/abstract_menu.hpp"

#include "gravity_autotune_menu.hpp"

namespace aruwsrc
{
class Drivers;
}  // namespace aruwsrc

/**
 * Weak function that you should define in `*_control.cpp` if an `gravityAutotuneCommand` exists.
 * Must be a null-terminated array of pointers to `GravityAutotuneBase` objects.
 */
aruwsrc::control::autotune::GravityAutotuneBase **getGravityAutotuneCommands();
namespace aruwsrc::display
{
class AutotuneMenu : public modm::AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView>>
{
public:
    /**
     * @param[in] vs `ViewStack` that this menu is sitting on top of.
     * @param[in] drivers A pointer to the global drivers object.
     */
    AutotuneMenu(
        modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView>> *vs,
        tap::Drivers *drivers,
        int entriesToDisplay);

    /**
     * Adds entries to the menu to the necessary submenus.
     */

    void draw() override;

    void update() override;

    bool hasChanged() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    static const char *getMenuName() { return "Autotune Calibrate Menu"; }

private:
    static constexpr int AUTOTUNE_MENU_ID = 15;
    static constexpr int DISPLAY_MAX_ENTRIES = 7;

    tap::Drivers *drivers;

    tap::display::VerticalScrollLogicHandler verticalScroll;

    uint8_t getCommandNumber() const { return getGravityAutotuneCommandNumber(); }

    uint8_t getGravityAutotuneCommandNumber() const
    {
        if (getGravityAutotuneCommands() == nullptr)
        {
            return 0;
        }
        else
        {
            uint8_t count = 0;
            while (getGravityAutotuneCommands()[count] != nullptr)
            {
                count++;
            }
            return count;
        }
    }
};
}  // namespace aruwsrc::display

#endif  // AUTOTUNE_MENU_HPP_
