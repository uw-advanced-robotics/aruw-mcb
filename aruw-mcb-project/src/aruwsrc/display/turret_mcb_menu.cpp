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

#include "tap/drivers.hpp"

namespace aruwsrc::display
{
TurretMCBMenu::TurretMCBMenu(
    modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> > *vs,
    aruwsrc::can::TurretMCBCanComm *turretMCBCanComm)
    : AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >(vs, TURRET_MCB_MENU_ID),
      turretMCBCanComm(turretMCBCanComm)
{
}

void TurretMCBMenu::draw()
{
    modm::GraphicDisplay &display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << getMenuName() << modm::endl;

    display << "Receiving Turret IMU data: " << turretMCBCanComm->isConnected() << modm::endl
            << "Limit switch depressed: " << turretMCBCanComm->getLimitSwitchDepressed()
            << modm::endl;
    display.printf(
        "Yaw (deg): %.2f\nYaw Velocity (deg/s): %.2f\nPitch (deg): %.2f\nPitch Velocity (deg/s): "
        "%.2f\n",
        static_cast<double>(modm::toDegree(turretMCBCanComm->getYaw())),
        static_cast<double>(modm::toDegree(turretMCBCanComm->getYawVelocity())),
        static_cast<double>(modm::toDegree(turretMCBCanComm->getPitch())),
        static_cast<double>(modm::toDegree(turretMCBCanComm->getPitchVelocity())));
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
