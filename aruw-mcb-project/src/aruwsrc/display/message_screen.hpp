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

#ifndef MESSAGE_SCREEN_HPP_
#define MESSAGE_SCREEN_HPP_

#include "tap/display/dummy_allocator.hpp"

#include "modm/ui/menu/abstract_menu.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc
{
namespace display
{
class MessageScreen : public modm::AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView>>
{
public:
    MessageScreen(modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView>> *vs, const char* msg);

    void draw() override;

    void update() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    bool hasChanged() override;

    static const char *getMenuName() { return "Crash Message"; }

private:
    const char* msg;
    static constexpr int MESSAGE_SCREEN_ID = 14;
};  // class ErrorMenu
}  // namespace display
}  // namespace aruwsrc

#endif  // ERROR_MENU_HPP_
