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

#include "trigger_homed_dual_joint_subsystem.hpp"

namespace aruwsrc::control::joint::homing
{
TriggerHomedDualJointSubsystem::TriggerHomedDualJointSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface& motorOne,
    tap::motor::MotorInterface& motorTwo,
    trigger::TriggerInterface& trigger,
    const tap::algorithms::SmoothPidConfig& configAlign,
    const TriggerHomedJointSubsystem::Config& config)
    : Subsystem(drivers),
      TriggerHomedJointSubsystem(
          drivers,
          motorOne,  // only left motor passed on bc we override everything that uses it anyway
                     // still have two motors stored locally
          trigger,
          config),
      motorOne(motorOne),
      motorTwo(motorTwo),
      alignPid(configAlign)
{
}

void TriggerHomedDualJointSubsystem::initialize()
{
    motorOne.initialize();
    motorTwo.initialize();

    resetEncoderValue();
}

void TriggerHomedDualJointSubsystem::resetEncoderValue()
{
    motorOne.getEncoder()->resetEncoderValue();
    motorTwo.getEncoder()->resetEncoderValue();
}

float TriggerHomedDualJointSubsystem::getPosition() const
{
    return (motorOne.getEncoder()->getPosition().getUnwrappedValue() +
            motorTwo.getEncoder()->getPosition().getUnwrappedValue()) /
           2 * encoderRatio;
}

float TriggerHomedDualJointSubsystem::getVelocity() const
{
    return (motorOne.getEncoder()->getVelocity() + motorTwo.getEncoder()->getVelocity()) / 2 *
           encoderRatio;
}

float TriggerHomedDualJointSubsystem::getPositionDifference()
{
    return (motorOne.getEncoder()->getPosition().getUnwrappedValue() -
            motorTwo.getEncoder()->getPosition().getUnwrappedValue()) *
           encoderRatio;
}

float TriggerHomedDualJointSubsystem::getVelocityDifference()
{
    return (motorOne.getEncoder()->getVelocity() - motorTwo.getEncoder()->getVelocity()) *
           encoderRatio;
}

void TriggerHomedDualJointSubsystem::runPosPidController(float dt)
{
    float error = setpoint.getValue() - getPosition();
    float outputPos = posPid.runController(error, getVelocity(), dt) + staticFeedforward;

    float errorAlignment = getPositionDifference();

    float outputAlign = alignPid.runController(errorAlignment, getVelocityDifference(), 2.0f);

    motorOne.setDesiredOutput(std::clamp(outputPos + outputAlign, -maxOutput, maxOutput));
    motorTwo.setDesiredOutput(std::clamp(outputPos - outputAlign, -maxOutput, maxOutput));
}

void TriggerHomedDualJointSubsystem::stopDuringHoming() { refreshSafeDisconnect(); }

void TriggerHomedDualJointSubsystem::refreshSafeDisconnect()
{
    motorOne.setDesiredOutput(0);
    motorTwo.setDesiredOutput(0);
}

}  // namespace aruwsrc::control::joint::homing
