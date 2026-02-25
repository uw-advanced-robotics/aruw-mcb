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

#include "fixed_path_auto_nav_command.hpp"

using aruwsrc::algorithms::AutoNavPath;

namespace aruwsrc
{
namespace control::chassis
{
FixedPathAutoNavCommand::FixedPathAutoNavCommand(
    const tap::Drivers& drivers,
    chassis::HolonomicChassisSubsystem& chassis,
    aruwsrc::control::chassis::ChassisAutoNavController& autoNavController,
    const Position* pathPoints,
    size_t numPathPoints,
    bool autoNavOnlyInGame,
    bool beybladeEnabled)
    : AutoNavCommand(
          drivers,
          chassis,
          autoNavController,
          autoNavOnlyInGame,
          beybladeEnabled,
          true),  // ends = true since this command should end when it reaches the end of the path
      autoNavController(autoNavController),
      path()
{
    for (int i = 0; i < numPathPoints; i++)
    {
        path.pushPoint(pathPoints[i]);
    }

    autoNavController.attachPath(&path);
}

bool FixedPathAutoNavCommand::isFinished() const
{
    return autoNavController
        .atSetpoint();  // TODO: add interupt thingy for if the control has an input
}
}  // namespace control::chassis
}  // namespace aruwsrc