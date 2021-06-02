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

#include "MainMenu.hpp"

namespace aruwlib
{
namespace display
{
MainMenu::MainMenu(modm::ViewStack<DummyAllocator<modm::IAbstractView> >* stack, Drivers* drivers)
    : modm::StandardMenu<DummyAllocator<modm::IAbstractView> >(stack, MAIN_MENU_ID),
      drivers(drivers),
      errorMenu(stack, drivers),
      hardwareTestMenu(stack, drivers),
      motorMenu(stack, drivers),
      commandSchedulerMenu(stack, drivers)
{
}

void MainMenu::initialize()
{
    addEntry(
        ErrorMenu::getMenuName(),
        modm::MenuEntryCallback<DummyAllocator<modm::IAbstractView> >(
            this,
            &MainMenu::addErrorMenuCallback));
    addEntry(
        HardwareTestMenu::getMenuName(),
        modm::MenuEntryCallback<DummyAllocator<modm::IAbstractView> >(
            this,
            &MainMenu::addHardwareTestMenuCallback));
    addEntry(
        MotorMenu::getMenuName(),
        modm::MenuEntryCallback<DummyAllocator<modm::IAbstractView> >(
            this,
            &MainMenu::addMotorMenuCallback));
    addEntry(
        "Property Table Menu",
        modm::MenuEntryCallback<DummyAllocator<modm::IAbstractView> >(
            this,
            &MainMenu::addPropertyTableCallback));
    addEntry(
        CommandSchedulerMenu::getMenuName(),
        modm::MenuEntryCallback<DummyAllocator<modm::IAbstractView> >(
            this,
            &MainMenu::addCommandSchedulerCallback));

    setTitle("Main Menu");
}

void MainMenu::addErrorMenuCallback()
{
    // em actually points to errorMenu
    ErrorMenu* em = new (&errorMenu) ErrorMenu(getViewStack(), drivers);
    getViewStack()->push(em);
}

void MainMenu::addHardwareTestMenuCallback()
{
    HardwareTestMenu* htm = new (&hardwareTestMenu) HardwareTestMenu(getViewStack(), drivers);
    getViewStack()->push(htm);
}

void MainMenu::addMotorMenuCallback()
{
    MotorMenu* mm = new (&motorMenu) MotorMenu(getViewStack(), drivers);
    getViewStack()->push(mm);
}

void MainMenu::addPropertyTableCallback()
{
    // TODO, see issue #221
}

void MainMenu::addCommandSchedulerCallback()
{
    CommandSchedulerMenu* csm =
        new (&commandSchedulerMenu) CommandSchedulerMenu(getViewStack(), drivers);
    getViewStack()->push(csm);
}
}  // namespace display

}  // namespace aruwlib
