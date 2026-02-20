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

#ifndef ERROR_SPECIFIC_MENU_HPP_
#define ERROR_SPECIFIC_MENU_HPP_

#include "tap/architecture/periodic_timer.hpp"

#include "modm/ui/menu/abstract_menu.hpp"
#include "tap/display/vertical_scroll_logic_handler.hpp"
#include "tap/drivers.hpp"

#include "tap/display/dummy_allocator.hpp"
#include "tap/errors/system_error.hpp"

namespace aruwsrc
{
namespace display
{
class ErrorSpecificMenu : public modm::AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >
{
public:
    ErrorSpecificMenu(
        modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> > *vs,
        tap::Drivers *drivers,
        const tap::errors::SystemError* currError);

    void draw() override;

    void update() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    bool hasChanged() override;

    static const char *getMenuName() { return "Error Specific Menu"; }


private:
    tap::Drivers* drivers;
    const tap::errors::SystemError* currError;
    std::string currDescription;
    int currLineNum;
    std::string currFile;
};
}  // namespace display
}  // namespace aruwsrc

#endif  // ERROR_SPECIFIC_MENU_HPP_
