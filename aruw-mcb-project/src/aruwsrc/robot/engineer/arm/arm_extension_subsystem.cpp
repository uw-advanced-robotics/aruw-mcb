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

#include "aruwsrc/robot/engineer/arm/arm_extension_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

namespace aruwsrc
{
namespace engineer
{
ArmExtensionSubsystem::ArmExtensionSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface& motors,
    const tap::algorithms::SmoothPidConfig& config,
    control::TriggerInterface& trigger,
    float radius,
    uint64_t length,
    float minSetpoint,
    float maxSetpoint,
    float kS,
    float epsilon)
    : LinearJointInterface(minSetpoint, maxSetpoint, epsilon),
      OneSidedBoundedSubsystemInterface(drivers, trigger, length),
      pid(config),
      motors(motors),
      radius(radius),
      kS(kS)
{
    this->setpoint = 0;
    this->homing = false;
}

void ArmExtensionSubsystem::initialize() { motors.initialize(); }

float ArmExtensionSubsystem::getPosition()
{
    return motors.getEncoder()->getPosition().getUnwrappedValue() * radius;
}

float ArmExtensionSubsystem::getVelocity() { return motors.getEncoder()->getVelocity() * radius; }

void ArmExtensionSubsystem::refresh()
{
    if (homing)
    {
        if (trigger.isTriggered())
        {
            motors.getEncoder()->resetEncoderValue();  // todo
            motors.setDesiredOutput(0);
            homing = false;
            return;
        }
        float errorPosition = -getPosition();

        float output = pid.runController(errorPosition, getVelocity(), 2.0f) + kS;
        motors.setDesiredOutput(output);  // todo
    }
    else
    {
        float errorPosition = setpoint - getPosition();

        float output = pid.runController(errorPosition, getVelocity(), 2.0f) + kS;
        motors.setDesiredOutput(output);
    }
}

void ArmExtensionSubsystem::refreshSafeDisconnect() { motors.setDesiredOutput(0); }

void ArmExtensionSubsystem::moveTowardLowerBound() {}

}  // namespace engineer
}  // namespace aruwsrc