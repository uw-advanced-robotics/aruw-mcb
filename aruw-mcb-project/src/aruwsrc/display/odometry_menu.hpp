/*
 * Copyright (c) 2024-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef ODOMETRY_MENU_HPP_
#define ODOMETRY_MENU_HPP_

#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/display/dummy_allocator.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/robot/engineer/algorithms/engineer_transforms.hpp"
#include "modm/ui/menu/abstract_menu.hpp"

namespace aruwsrc
{
class Drivers;
}
namespace aruwsrc::display
{
class OdometryMenu : public modm::AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView>>
{
public:
    OdometryMenu(
        modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView>>* stack,
        tap::algorithms::odometry::Odometry2DInterface* odometry);

    void draw() override;
    void update() override;
    bool hasChanged() override;
    void shortButtonPress(modm::MenuButtons::Button button) override;

    static const char* getMenuName() { return "Odometry Menu"; }

private:
    static constexpr int ODOMETRY_MENU_ID = 20;

    tap::algorithms::odometry::Odometry2DInterface* odometry;
};
}  // namespace aruwsrc::display
#endif