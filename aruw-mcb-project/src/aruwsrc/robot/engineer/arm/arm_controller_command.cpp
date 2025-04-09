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

#include "aruwsrc/robot/engineer/arm/arm_controller_command.hpp"

namespace aruwsrc::control::engineer
{
using namespace aruwsrc::engineer;

JointControllerCommand::JointControllerCommand(
    ArmLiftSubsystem &lift,
    ArmExtensionSubsystem &extension,
    JointSubsystem &wristRoll,
    WristSubsystem &wrist,
    EngineerControlOperatorInterface *operatorInterface,
    float liftScalingFactor,
    float extensionScalingFactor,
    float wristRollScalingFactor,
    float wristPitchScalingFactor,
    float wristYawScalingFactor)
    : tap::control::Command(),
      lift(lift),
      extension(extension),
      wristRoll(wristRoll),
      wrist(wrist),
      operatorInterface(operatorInterface),
      liftScalingFactor(liftScalingFactor),
      extensionScalingFactor(extensionScalingFactor),
      wristRollScalingFactor(wristRollScalingFactor),
      wristPitchScalingFactor(wristPitchScalingFactor),
      wristYawScalingFactor(wristYawScalingFactor)
{
    addSubsystemRequirement(&lift);
    addSubsystemRequirement(&extension);
    addSubsystemRequirement(&wristRoll);
    addSubsystemRequirement(&wrist);
}

void JointControllerCommand::initialize() {}

void JointControllerCommand::execute()
{
    // Get the desired velocities from the operator interface
    float liftVelocity = operatorInterface->getArmLiftVelocity() * liftScalingFactor;
    float extensionVelocity = operatorInterface->getArmExtensionVelocity() * extensionScalingFactor;
    float wristRollVelocity = operatorInterface->getArmWristRollVelocity() * wristRollScalingFactor;
    float wristPitchVelocity =
        operatorInterface->getArmWristPitchVelocity() * wristPitchScalingFactor;
    float wristYawVelocity = operatorInterface->getArmWristYawVelocity() * wristYawScalingFactor;

    // Set the desired velocities
    lift.setSetpoint(lift.getSetpoint() + liftVelocity);
    extension.setSetpoint(extension.getSetpoint() + extensionVelocity);
    wristRoll.setSetpoint(wristRoll.getSetpoint() + wristRollVelocity);
    wrist.setSetpointPitch(wrist.getSetpointPitch() + wristPitchVelocity);
    wrist.setSetpointYaw(wrist.getSetpointYaw() + wristYawVelocity);
}

}  // namespace aruwsrc::control::engineer