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

#include "tap/algorithms/math_user_utils.hpp"

namespace aruwsrc
{
namespace engineer
{
ArmLiftSubsystem::ArmLiftSubsystem(
    tap::Drivers* drivers,
    tap::motor::DjiMotor& motorLeft,
    tap::motor::DjiMotor& motorRight,
    tap::algorithms::SmoothPidConfig& config,
    float radius,
    float minSetpoint,
    float maxSetpoint,
    float kS,
    float epsilon)
    : LinearJointInterface(drivers, epsilon, minSetpoint, maxSetpoint),
      pidPos(config),
      pidAlign(config),
      motorLeft(motorLeft),
      motorRight(motorRight),
      radius(radius),
      kS(kS)
{
    this->setpoint = 0;
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

void ArmLiftSubsystem::refresh()
{
    float errorPosition = setpoint - getPosition();
    float errorAlignment = getPositionDifference();

    float outputPos = pidPos.runController(errorPosition, getAverageVelocity(), 2.0f) + kS;
    float outputAlign = pidAlign.runController(errorAlignment, getVelocityDifference(), 2.0f);

    motorLeft.setDesiredOutput(outputPos + outputAlign);
    motorRight.setDesiredOutput(outputPos - outputAlign);
}

void ArmLiftSubsystem::refreshSafeDisconnect()
{
    motorLeft.setDesiredOutput(0);
    motorRight.setDesiredOutput(0);
}

}  // namespace engineer
}  // namespace aruwsrc