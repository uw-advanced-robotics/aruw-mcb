/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "aruwsrc/robot/engineer/arm/joint_controller_command.hpp"

namespace aruwsrc
{
namespace engineer
{
JointControllerCommand::JointControllerCommand(
    JointSubsystem &joint,
    control::ControlOperatorInterface *operatorInterface,
    float scalingFactor)
    : tap::control::Command(),
      joint(joint),
      operatorInterface(operatorInterface),
      scalingFactor(scalingFactor)
{
    addSubsystemRequirement(&joint);
}

void JointControllerCommand::initialize()
{
    // todo use custom chassisOperatorInterface for engineer
    joint.setSetpoint(operatorInterface->getChassisYInput() * scalingFactor);
}

void JointControllerCommand::execute()
{
    joint.setSetpoint(operatorInterface->getChassisYInput() * scalingFactor);
}

}  // namespace engineer
}  // namespace aruwsrc