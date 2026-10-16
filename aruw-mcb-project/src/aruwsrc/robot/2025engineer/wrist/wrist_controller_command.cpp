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

#include "aruwsrc/robot/2025engineer/wrist/wrist_controller_command.hpp"

namespace aruwsrc::engineer::wrist
{
WristControllerCommand::WristControllerCommand(
    aruwsrc::control::joint::JointSubsystem &roll,
    WristSubsystem &wrist,
    EngineerControlOperatorInterface *operatorInterface,
    float rollScalingFactor,
    float pitchScalingFactor,
    float yawScalingFactor)
    : tap::control::Command(),
      roll(roll),
      wrist(wrist),
      operatorInterface(operatorInterface),
      rollScalingFactor(rollScalingFactor),
      pitchScalingFactor(pitchScalingFactor),
      yawScalingFactor(yawScalingFactor)
{
    addSubsystemRequirement(&roll);
    addSubsystemRequirement(&wrist);
}

void WristControllerCommand::initialize() {}

void WristControllerCommand::execute()
{
    // Get the desired velocities from the operator interface
    // to add/subtract from position setpoint
    float rollVelocity = operatorInterface->getWristRollVelocity() * rollScalingFactor;
    float pitchVelocity = operatorInterface->getWristPitchVelocity() * pitchScalingFactor;
    float yawVelocity = operatorInterface->getWristYawVelocity() * yawScalingFactor;

    // Set the desired positions
    roll.setSetpoint(roll.getSetpoint() + rollVelocity);
    wrist.setSetpointPitch(wrist.getSetpointPitch() + pitchVelocity);
    wrist.setSetpointYaw(wrist.getSetpointYaw() + yawVelocity);
}

}  // namespace aruwsrc::engineer::wrist