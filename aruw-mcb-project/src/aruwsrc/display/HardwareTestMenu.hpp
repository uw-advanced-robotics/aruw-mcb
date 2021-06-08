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

#ifndef HARDWARE_TEST_MENU_HPP_
#define HARDWARE_TEST_MENU_HPP_

#include <modm/ui/menu/abstract_menu.hpp>

#include "aruwlib/control/command_scheduler_types.hpp"
#include "aruwlib/display/VerticalScrollLogicHandler.hpp"

namespace aruwsrc
{
class Drivers;

namespace display
{
class HardwareTestMenu : public modm::AbstractMenu
{
public:
    HardwareTestMenu(modm::ViewStack *vs, aruwlib::Drivers *drivers);

    void draw() override;

    void update() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    bool hasChanged() override;

    static const char *getMenuName() { return "Hardware Test Menu"; }

private:
    static constexpr int HARDWARE_TEST_MENU_ID = 4;
    static constexpr int MAX_ENTRIES_DISPLAYED = 6;

    aruwlib::Drivers *drivers;

    aruwlib::control::subsystem_scheduler_bitmap_t completeSubsystems = 0;

    aruwlib::display::VerticalScrollLogicHandler vertScrollHandler;
};  // class HardwareTestMenu
}  // namespace display
}  // namespace aruwsrc

#endif  // HARDWARE_TEST_MENU_HPP_
