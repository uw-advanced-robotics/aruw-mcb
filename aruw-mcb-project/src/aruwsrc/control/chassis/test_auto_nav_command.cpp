/*
 * Copyright (c) 2020-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "test_auto_nav_command.hpp"

namespace aruwsrc::control::chassis
{
TestAutoNavCommand::TestAutoNavCommand(
    const tap::Drivers& drivers,
    chassis::HolonomicChassisSubsystem& chassis,
    aruwsrc::control::chassis::ChassisAutoNavController& autoNavController,
    std::vector<Transform> transforms = std::vector<Transform>(),
    tap::algorithms::odometry::Odometry2DInterface* odometrySubsystem = nullptr,
    bool autoNavOnlyInGame = false,
    bool beybladeEnabled = true,
    bool ends = false)
    : drivers(drivers),
      autoNavController(autoNavController),
      transforms(transforms),
      odometrySubsystem(odometrySubsystem),
      AutoNavCommand(drivers, chassis, autoNavController, autoNavOnlyInGame, beybladeEnabled, ends)
{
}

void TestAutoNavCommand::initialize()
{
    AutoNavCommand::initialize();
    Position currPosition = Position(
        odometrySubsystem->getCurrentLocation2D().getX(),
        odometrySubsystem->getCurrentLocation2D().getY(),
        0.0f);
    for (Transform transform : transforms)
    {
        currPosition = transform.apply(currPosition);
        path.pushPoint(currPosition);
    }
    autoNavController.attachPath(&path);
}

bool TestAutoNavCommand::isFinished() const { return true; }
}  // namespace aruwsrc::control::chassis