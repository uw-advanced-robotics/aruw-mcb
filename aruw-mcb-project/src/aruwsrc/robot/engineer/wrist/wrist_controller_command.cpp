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

#include "aruwsrc/robot/engineer/wrist/wrist_controller_command.hpp"

namespace aruwsrc::engineer::wrist
{
WristControllerCommand::WristControllerCommand(
    WristSubsystem &wrist,
    EngineerControlOperatorInterface *operatorInterface,
    float theta1ScalingFactor,
    float theta2ScalingFactor,
    float theta3ScalingFactor)
    : tap::control::Command(),
      wrist(wrist),
      operatorInterface(operatorInterface),
      theta1ScalingFactor(theta1ScalingFactor),
      theta2ScalingFactor(theta2ScalingFactor),
      theta3ScalingFactor(theta3ScalingFactor)
{
    addSubsystemRequirement(&wrist);
}

void WristControllerCommand::initialize()
{
    wrist.setSetpointTheta1(wrist.getTheta1());
    wrist.setSetpointTheta2(wrist.getTheta2());
    wrist.setSetpointTheta3(wrist.getTheta3());
}

void WristControllerCommand::execute()
{
    // Get the desired velocities from the operator interface
    // to add/subtract from position setpoint
    float theta1Velocity = operatorInterface->getWristTheta1Velocity() * theta1ScalingFactor;
    float theta2Velocity = operatorInterface->getWristTheta2Velocity() * theta2ScalingFactor;
    float theta3Velocity = operatorInterface->getWristTheta3Velocity() * theta3ScalingFactor;

    // Set the desired positions
    wrist.setSetpointTheta3(wrist.getSetpointTheta3() + theta3Velocity);  // theta3 is "roll"
    wrist.setSetpointTheta2(wrist.getSetpointTheta2() + theta2Velocity);  // theta2 is "pitch"
    wrist.setSetpointTheta1(wrist.getSetpointTheta1() + theta1Velocity);  // theta1 is "yaw"
}

}  // namespace aruwsrc::engineer::wrist