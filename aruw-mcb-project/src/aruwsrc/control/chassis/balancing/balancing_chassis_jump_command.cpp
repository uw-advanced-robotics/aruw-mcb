/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "balancing_chassis_jump_command.hpp"

#include <cassert>

namespace aruwsrc
{
namespace chassis
{
BalancingChassisJumpCommand::BalancingChassisJumpCommand(
    tap::Drivers* drivers,
    BalancingChassisSubsystem* chassis)
    : drivers(drivers),
      chassis(chassis)
{
    assert(chassis != nullptr);
    addSubsystemRequirement(chassis);
}

void BalancingChassisJumpCommand::initialize()
{
    if (chassis->getArmState())
    {
        chassis->startJumping();
    }
    else
    {
        end(false);
    }
}

void BalancingChassisJumpCommand::execute() { end(false); }

void BalancingChassisJumpCommand::end(bool interrupted) {}

bool BalancingChassisJumpCommand::isFinished() const { return true; }
}  // namespace chassis
}  // namespace aruwsrc