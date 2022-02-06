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

#include "turret_mcb_menu.hpp"

#include "aruwsrc/drivers.hpp"

namespace aruwsrc::display
{
TurretMCBMenu::TurretMCBMenu(
    modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> > *vs,
    aruwsrc::Drivers *drivers)
    : AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >(vs, IMU_CALIBRATE_MENU_ID),
      drivers(drivers)
{
}

void TurretMCBMenu::draw()
{
    modm::GraphicDisplay &display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << getMenuName() << modm::endl;

    display << "Receiving Turret IMU data " << drivers->turretMCBCanComm.isConnected()
            << modm::endl;
    display << "Limit switch depressed" << drivers->turretMCBCanComm.getLimitSwitchDepressed()
            << modm::endl;
    display.printf(
        "Yaw: %.2f\nYaw Velocity: %.2f\nPitch: %.2f\nPitch Velocity: %.2f",
        drivers->turretMCBCanComm.getYaw(),
        drivers->turretMCBCanComm.getYawVelocity(),
        drivers->turretMCBCanComm.getPitch(),
        drivers->turretMCBCanComm.getPitchVelocity());
}

void TurretMCBMenu::update() {}

void TurretMCBMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    if (button == modm::MenuButtons::LEFT)
    {
        this->remove();
    }
}

bool TurretMCBMenu::hasChanged() { return updatePeriodicTimer.execute(); }

}  // namespace aruwsrc::display
