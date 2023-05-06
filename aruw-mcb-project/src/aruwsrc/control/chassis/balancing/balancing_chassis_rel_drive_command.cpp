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

#include "balancing_chassis_rel_drive_command.hpp"

#include <cassert>

namespace aruwsrc
{
namespace chassis
{
BalancingChassisRelativeDriveCommand::BalancingChassisRelativeDriveCommand(
    tap::Drivers* drivers,
    BalancingChassisSubsystem* chassis,
    control::ControlOperatorInterface& operatorInterface)
    : drivers(drivers),
      chassis(chassis),
      operatorInterface(operatorInterface)
{
    assert(chassis != nullptr);
    addSubsystemRequirement(chassis);
}

void BalancingChassisRelativeDriveCommand::initialize() { chassis->setDesiredOutput(0, 0); }

void BalancingChassisRelativeDriveCommand::execute()
{
    chassis->setDesiredOutput(
        operatorInterface.getChassisXInput() * TRANSLATION_REMOTE_SCALAR,
        -operatorInterface.getChassisYInput() * ROTATION_REMOTE_SCALAR);
    chassis->setDesiredHeight(
        0.01 * drivers->remote.getChannel(tap::communication::serial::Remote::Channel::WHEEL));
}

void BalancingChassisRelativeDriveCommand::end(bool interrupted)
{
    chassis->setDesiredOutput(0, 0);
}

bool BalancingChassisRelativeDriveCommand::isFinished() const { return false; }
}  // namespace chassis
}  // namespace aruwsrc