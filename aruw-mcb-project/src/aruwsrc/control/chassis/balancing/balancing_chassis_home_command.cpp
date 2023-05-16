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

#include "balancing_chassis_home_command.hpp"

#include <cassert>

namespace aruwsrc
{
namespace chassis
{
BalancingChassisHomeCommand::BalancingChassisHomeCommand(
    tap::Drivers* drivers,
    BalancingChassisSubsystem* chassis)
    : drivers(drivers),
      chassis(chassis)
{
    assert(chassis != nullptr);
    addSubsystemRequirement(chassis);
}

void BalancingChassisHomeCommand::initialize()
{
    chassis->stopChassis();
}

void BalancingChassisHomeCommand::execute() { chassis->startHoming(); }

void BalancingChassisHomeCommand::end(bool interrupted)
{
    chassis->setDesiredOutput(0, 0);
    chassis->armChassis();
}

bool BalancingChassisHomeCommand::isFinished() const { return false; }
}  // namespace chassis
}  // namespace aruwsrc