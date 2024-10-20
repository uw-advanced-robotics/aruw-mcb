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

namespace aruwsrc
{
namespace display
{
FLASH_STORAGE(uint8_t aruwImage[]) = {
    128, 64,  0,   0,   0,   0,   0,   0,   0,   0,   0,   128, 248, 240, 224, 192, 128, 0,   0,
    0,   0,   192, 248, 254, 248, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   128, 128, 192, 192, 192, 224, 224, 240, 240, 248, 248,
    248, 248, 240, 0,   0,   0,   0,   0,   192, 192, 192, 192, 192, 192, 192, 192, 192, 192, 192,
    192, 192, 192, 192, 192, 192, 192, 192, 192, 192, 192, 0,   0,   0,   0,   0,   0,   0,   0,
    192, 192, 192, 192, 192, 192, 192, 192, 0,   0,   0,   0,   0,   0,   0,   0,   192, 192, 192,
    192, 192, 192, 192, 192, 192, 192, 192, 192, 192, 192, 192, 192, 0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   255, 255, 255, 255, 1,   3,   7,   14,  28,  56,  115, 247, 243,
    249, 253, 252, 252, 252, 252, 254, 254, 255, 255, 127, 127, 127, 127, 63,  63,  63,  62,  190,
    254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 63,  31,  7,   0,   0,
    0,   0,   0,   255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 0,   0,   0,   0,   0,   0,   0,   248, 255, 255, 255, 255, 255,
    255, 255, 255, 248, 0,   0,   0,   0,   0,   0,   0,   255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 0,   0,   0,   0,   0,   128, 128, 128, 128, 192, 192,
    252, 252, 249, 243, 231, 79,  30,  252, 240, 240, 248, 254, 255, 255, 255, 255, 255, 255, 31,
    1,   241, 252, 12,  192, 248, 120, 0,   128, 248, 252, 254, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 127, 31,  15,  7,   3,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   31,  255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 224, 0,   0,   0,
    0,   0,   0,   0,   0,   0,   224, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    224, 0,   0,   0,   0,   0,   0,   0,   0,   128, 252, 255, 255, 255, 255, 255, 255, 63,  3,
    0,   0,   0,   0,   0,   0,   0,   7,   15,  31,  31,  255, 255, 255, 255, 255, 255, 255, 7,
    224, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 15,  0,   224, 255, 15,  192, 252, 127,
    15,  128, 248, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 252, 254, 254,
    254, 254, 254, 255, 127, 127, 127, 127, 63,  15,  0,   0,   0,   0,   0,   0,   0,   0,   0,
    3,   31,  255, 255, 255, 255, 255, 255, 255, 255, 255, 252, 128, 0,   0,   0,   0,   0,   128,
    252, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 252, 128, 0,   0,
    0,   0,   0,   240, 255, 255, 255, 255, 255, 255, 63,  3,   0,   0,   0,   0,   0,   0,   0,
    0,   28,  63,  63,  63,  127, 255, 255, 255, 255, 255, 255, 255, 240, 128, 31,  255, 255, 255,
    255, 255, 255, 31,  15,  193, 120, 30,  135, 240, 252, 63,  15,  192, 252, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 251, 227, 195, 3,   1,   1,   1,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   3,   63,  255,
    255, 255, 255, 255, 255, 255, 255, 255, 240, 0,   0,   128, 240, 255, 255, 255, 255, 127, 7,
    0,   7,   127, 255, 255, 255, 255, 255, 255, 255, 255, 255, 248, 128, 0,   192, 254, 255, 255,
    255, 255, 255, 63,  3,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   112, 248,
    252, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 248, 241, 227, 143, 143, 1,   96,  126,
    7,   128, 252, 255, 15,  199, 240, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 254, 248, 240, 192, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   15,  255, 255, 255, 255, 255,
    255, 255, 255, 255, 254, 248, 255, 255, 255, 255, 255, 15,  0,   0,   0,   0,   0,   15,  255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 248, 255, 255, 255, 255, 255, 63,  3,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,
    1,   1,   3,   255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 252, 252, 249, 249, 248, 240,
    243, 243, 135, 7,   255, 255, 255, 63,  63,  127, 127, 255, 255, 255, 255, 255, 195, 3,   7,
    15,  15,  31,  63,  127, 248, 240, 128, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   7,   255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 63,  1,   0,   0,   0,   0,   0,   0,   0,   1,   63,  255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 127, 7,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   3,   7,
    7,   7,   3,   1,   1,   1,   1,   3,   3,   255, 255, 255, 255, 255, 31,  31,  15,  0,   15,
    255, 127, 0,   0,   0,   0,   0,   1,   3,   7,   15,  63,  0,   0,   0,   0,   0,   0,   0,
    0,   3,   7,   14,  24,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   7,   7,   7,   7,   7,   7,   7,   7,   7,
    7,   7,   7,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0};

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
    display.drawImage(modm::glcd::Point(0, 0), modm::accessor::asFlash(aruwImage));

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
