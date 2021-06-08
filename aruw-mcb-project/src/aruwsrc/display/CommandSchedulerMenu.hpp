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

#ifndef COMMAND_SCHEDULER_MENU_HPP_
#define COMMAND_SCHEDULER_MENU_HPP_

#include <modm/ui/menu/abstract_menu.hpp>

#include "aruwlib/architecture/periodic_timer.hpp"
#include "aruwlib/control/command_scheduler_types.hpp"
#include "aruwlib/display/VerticalScrollLogicHandler.hpp"

namespace aruwsrc
{
class Drivers;

namespace control
{
class Command;
class Subsystem;
}  // namespace control

namespace display
{
class CommandSchedulerMenu : public modm::AbstractMenu
{
public:
    CommandSchedulerMenu(modm::ViewStack *stack, aruwlib::Drivers *drivers);

    ~CommandSchedulerMenu() = default;

    void draw() override;

    void update() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    bool hasChanged() override;

    static const char *getMenuName() { return "Command Scheduler"; }

private:
    static constexpr int MAX_ENTRIES_DISPLAYED = 5;

    aruwlib::Drivers *drivers;
    aruwlib::display::VerticalScrollLogicHandler vertScrollHandler;
    bool firstDrawTime;
    aruwlib::control::subsystem_scheduler_bitmap_t prevRegisteredSubsystems = 0;
    aruwlib::control::command_scheduler_bitmap_t prevAddedCommands = 0;
};  // class CommandSchedulerMenu
}  // namespace display
}  // namespace aruwsrc

#endif  // COMMAND_SCHEDULER_MENU_HPP_
