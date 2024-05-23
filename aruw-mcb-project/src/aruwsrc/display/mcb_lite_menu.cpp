/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "mcb_lite_menu.hpp"

#include "tap/drivers.hpp"

#include "aruwsrc/communication/mcb-lite/mcb_lite.hpp"

namespace aruwsrc::display
{
MCBLiteMenu::MCBLiteMenu(
    modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> > *vs,
    aruwsrc::virtualMCB::MCBLite *mcbLite)
    : AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >(vs, MCB_LITE_MENU_ID),
      mcbLite(mcbLite)
{
}

void MCBLiteMenu::draw()
{
    modm::GraphicDisplay &display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << getMenuName() << modm::endl;

    display.printf(
        "IMU data: Calibrated? %d (yaw): %.2f\n",
        mcbLite->imu.getImuState() ==
            tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATED,
        static_cast<double>(mcbLite->imu.getYaw()));

    display << "Motor positions: " << modm::endl;
    // Position of the motors, should be the first two bytes
    for (int i = 0; i < 8; i++)
    {
        display << "Motor " << i << ": "
                << (mcbLite->can1Data[i * 8] << 8 | mcbLite->can1Data[i * 8 + 1]) << "    ";
        i++;
        display << "Motor " << i << ": "
                << (mcbLite->can1Data[i * 8] << 8 | mcbLite->can1Data[i * 8 + 1]) << modm::endl;
    }
}

void MCBLiteMenu::update() {}

void MCBLiteMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    if (button == modm::MenuButtons::LEFT)
    {
        this->remove();
    }
}

bool MCBLiteMenu::hasChanged() { return updatePeriodicTimer.execute(); }

}  // namespace aruwsrc::display
