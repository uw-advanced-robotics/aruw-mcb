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
    tap::algorithms::SmoothPidConfig& config,
    float radius,
    float minSetpoint,
    float maxSetpoint,
    float kS,
    float epsilon)
    : LinearJointInterface(epsilon, minSetpoint, maxSetpoint),
      tap::control::Subsystem(drivers),
      pid(config),
      motors(motors),
      radius(radius),
      kS(kS)
{
    this->setpoint = 0;
}

float ArmExtensionSubsystem::getPosition()
{
    return motors.getEncoder()->getPosition().getUnwrappedValue() * radius;
}

float ArmExtensionSubsystem::getVelocity() { return motors.getEncoder()->getVelocity() * radius; }

void ArmExtensionSubsystem::refresh()
{
    float errorPosition = setpoint - getPosition();

    float output = pid.runController(errorPosition, getVelocity(), 2.0f) + kS;

    motors.setDesiredOutput(output);
}

void ArmExtensionSubsystem::refreshSafeDisconnect() { motors.setDesiredOutput(0); }

}  // namespace engineer
}  // namespace aruwsrc