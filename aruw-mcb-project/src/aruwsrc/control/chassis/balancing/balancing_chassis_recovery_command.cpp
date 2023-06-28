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

#include "balancing_chassis_recovery_command.hpp"

#include <cassert>

namespace aruwsrc
{
namespace chassis
{
BalancingChassisRecoveryCommand::BalancingChassisRecoveryCommand(
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

void BalancingChassisRecoveryCommand::initialize()
{
    chassis->startRecovery();
    chassis->setDesiredOutput(0, 0);
}

void BalancingChassisRecoveryCommand::execute()
{
    if (!chassis->getArmState()) chassis->armChassis();

    chassis->setDesiredOutput(
        operatorInterface.getChassisXInput() * TRANSLATION_REMOTE_SCALAR,
        -operatorInterface.getChassisYInput() * ROTATION_REMOTE_SCALAR);
}

void BalancingChassisRecoveryCommand::end(bool interrupted)
{
    chassis->endRecovery();
    chassis->setDesiredOutput(0, 0);
    chassis->disarmChassis();
}

bool BalancingChassisRecoveryCommand::isFinished() const { return false; }
}  // namespace chassis
}  // namespace aruwsrc