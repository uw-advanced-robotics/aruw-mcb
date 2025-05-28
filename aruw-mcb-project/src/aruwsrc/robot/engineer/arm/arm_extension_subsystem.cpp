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
    tap::motor::MotorInterface& motor,
    const tap::algorithms::SmoothPidConfig& config,
    control::TriggerInterface& trigger,
    float radius,
    float lowerBound,
    float upperBound,
    float epsilon)
    : LimitSwitchSetpointInterface(drivers, trigger, lowerBound, upperBound, epsilon),
      pid(config),
      motor(motor),
      radius(radius)
{
    this->setpoint = 0;
    this->home = 0;
}

void ArmExtensionSubsystem::initialize() { motor.initialize(); }

float ArmExtensionSubsystem::getPosition()
{
    return motor.getEncoder()->getPosition().getUnwrappedValue() * radius;
}

float ArmExtensionSubsystem::getVelocity() { return motor.getEncoder()->getVelocity() * radius; }

void ArmExtensionSubsystem::refresh()
{
    if (calibrationState == CalibrationState::CALIBRATING_LOWER_BOUND)
    {
        if (trigger.isTriggered())
        {
            calibrationState = CalibrationState::CALIBRATION_COMPLETE;
            motor.getEncoder()->resetEncoderValue();
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

        float output = pid.runController(errorPosition, getVelocity(), 2.0f);
        motor.setDesiredOutput(output);
    }
}

void ArmExtensionSubsystem::refreshSafeDisconnect() { motor.setDesiredOutput(0); }

void ArmExtensionSubsystem::moveTowardLowerBound()
{
    motor.setDesiredOutput(-1000.0f);  // todo
}

void ArmExtensionSubsystem::stopDuringHoming()
{
    motor.setDesiredOutput(0);  // todo
}

}  // namespace engineer
}  // namespace aruwsrc