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

#include "aruwsrc/robot/engineer/arm/arm_lift_subsystem.hpp"
namespace aruwsrc
{
namespace engineer
{
ArmLiftSubsystem::ArmLiftSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface& motorLeft,
    tap::motor::MotorInterface& motorRight,
    const tap::algorithms::SmoothPidConfig& configPos,
    const tap::algorithms::SmoothPidConfig& configAlign,
    control::TriggerInterface& trigger,
    float radius,
    uint64_t length,
    float minSetpoint,
    float maxSetpoint,
    float kS,
    float epsilon)
    : OneSidedBoundedSubsystemInterface(drivers, trigger, length),
      LinearJointInterface(minSetpoint, maxSetpoint, epsilon),
      pidPos(configPos),
      pidAlign(configAlign),
      motorLeft(motorLeft),
      motorRight(motorRight),
      trigger(trigger),
      radius(radius),
      kS(kS)
{
    this->setpoint = 0;
    this->home = 0;
}

float ArmLiftSubsystem::getPosition()
{
    return (motorLeft.getEncoder()->getPosition().getUnwrappedValue() +
            motorRight.getEncoder()->getPosition().getUnwrappedValue()) *
           radius / 2;
}

float ArmLiftSubsystem::getPositionDifference()
{
    return (motorLeft.getEncoder()->getPosition().getUnwrappedValue() -
            motorRight.getEncoder()->getPosition().getUnwrappedValue()) *
           radius;
}

float ArmLiftSubsystem::getAverageVelocity()
{
    return (motorLeft.getEncoder()->getVelocity() + motorRight.getEncoder()->getVelocity()) *
           radius / 2;
}

float ArmLiftSubsystem::getVelocityDifference()
{
    return (motorLeft.getEncoder()->getVelocity() - motorRight.getEncoder()->getVelocity()) *
           radius;
}

void ArmLiftSubsystem::initialize()
{
    motorLeft.initialize();
    motorRight.initialize();

    motorLeft.getEncoder()->resetEncoderValue();
    motorRight.getEncoder()->resetEncoderValue();
}

void ArmLiftSubsystem::refresh()
{
    if (calibrationState == CalibrationState::CALIBRATING_LOWER_BOUND)
    {
        if (trigger.isTriggered())
        {
            calibrationState = CalibrationState::CALIBRATION_COMPLETE;
            motorLeft.getEncoder()->resetEncoderValue();
            motorRight.getEncoder()->resetEncoderValue();
            setSetpoint(home);
        }
        else
        {
            moveTowardLowerBound();
        }
    }
    else
    {
        float errorPosition = setpoint - getPosition();

        float errorAlignment = getPositionDifference();

        float outputPos = pidPos.runController(errorPosition, getAverageVelocity(), 2.0f) + kS;
        float outputAlign = pidAlign.runController(errorAlignment, getVelocityDifference(), 2.0f);

        motorLeft.setDesiredOutput(outputPos + outputAlign);
        motorRight.setDesiredOutput(outputPos - outputAlign);
    }
}

void ArmLiftSubsystem::refreshSafeDisconnect()
{
    motorLeft.setDesiredOutput(0);
    motorRight.setDesiredOutput(0);
}

void ArmLiftSubsystem::moveTowardLowerBound()
{
    float errorAlignment = getPositionDifference();

    float outputAlign = pidAlign.runController(errorAlignment, getVelocityDifference(), 2.0f);

    motorLeft.setDesiredOutput(1000 + outputAlign);
    motorRight.setDesiredOutput(1000 - outputAlign);  // todo
}

void ArmLiftSubsystem::stopDuringHoming() { refreshSafeDisconnect(); }

}  // namespace engineer
}  // namespace aruwsrc