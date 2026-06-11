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

#include "odometry_menu.hpp"

namespace aruwsrc::display
{
OdometryMenu::OdometryMenu(
    modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView>>* stack,
    tap::algorithms::odometry::Odometry2DInterface* odometry)
    : AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView>>(stack, ODOMETRY_MENU_ID),
      odometry(odometry)
{
}

void OdometryMenu::draw()
{
    modm::GraphicDisplay& display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << getMenuName() << modm::endl;

    auto loc = odometry->getCurrentLocation2D();
    auto vel = odometry->getCurrentVelocity2D();
    display << "Position (x,y): " << loc.getX() << ", " << loc.getY() << modm::endl;
    display << "Orientation (yaw): " << odometry->getYaw() << modm::endl;
    display << "Velocity (x,y): " << vel.x << ", " << vel.y << modm::endl;
    display << "Last Computed Time: " << odometry->getLastComputedOdometryTime() << modm::endl;
}

void OdometryMenu::update() {}

bool OdometryMenu::hasChanged() { return true; }

void OdometryMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    if (button == modm::MenuButtons::LEFT) this->remove();
}
}  // namespace aruwsrc::display