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

#include "splash_screen.hpp"

#include "main_menu.hpp"
#include "splash_screen_logo.hpp"

namespace aruwsrc
{
namespace display
{
SplashScreen::SplashScreen(
    modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> >* vs,
    tap::Drivers* drivers,
    serial::VisionCoprocessor* visionCoprocessor,
    can::TurretMCBCanComm* turretMCBCanCommBus1,
    can::TurretMCBCanComm* turretMCBCanCommBus2,
    aruwsrc::virtualMCB::MCBLite* mcbLite1,
    aruwsrc::virtualMCB::MCBLite* mcbLite2,
    can::capbank::CapacitorBank* capacitorBank)
    : modm::AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >(
          vs,
          SPLASH_SCREEN_MENU_ID),
      drivers(drivers),
      visionCoprocessor(visionCoprocessor),
      turretMCBCanCommBus1(turretMCBCanCommBus1),
      turretMCBCanCommBus2(turretMCBCanCommBus2),
      mcbLite1(mcbLite1),
      mcbLite2(mcbLite2),
      capacitorBank(capacitorBank)
{
}

void SplashScreen::draw()
{
    modm::GraphicDisplay& display = getViewStack()->getDisplay();
    display.clear();
    display.drawImage(
        modm::glcd::Point(0, 0),
#ifdef SSH1106_OLED
        modm::accessor::asFlash(ARUW_IMAGE_128X64)
#else
        modm::accessor::asFlash(ARUW_IMAGE_128X128)
#endif
    );

    drawn = true;
}

void SplashScreen::update() {}

void SplashScreen::shortButtonPress(modm::MenuButtons::Button button)
{
    switch (button)
    {
        case modm::MenuButtons::LEFT:
            break;
        case modm::MenuButtons::RIGHT:
        {
            MainMenu* mm = new MainMenu(
                getViewStack(),
                drivers,
                visionCoprocessor,
                turretMCBCanCommBus1,
                turretMCBCanCommBus2,
                mcbLite1,
                mcbLite2,
                capacitorBank);
            mm->initialize();
            getViewStack()->push(mm);
            break;
        }
        case modm::MenuButtons::DOWN:
            break;
        case modm::MenuButtons::UP:
            break;
        case modm::MenuButtons::OK:
            break;
    }
}

bool SplashScreen::hasChanged() { return !drawn; }
}  // namespace display
}  // namespace aruwsrc
