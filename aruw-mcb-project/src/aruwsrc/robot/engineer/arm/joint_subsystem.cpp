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

#include "aruwsrc/robot/engineer/arm/joint_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

namespace aruwsrc
{
namespace engineer
{
JointSubsystem::JointSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface& motor,
    const tap::algorithms::SmoothPidConfig& config,
    float lowerBound,
    float upperBound,
    float kS,
    float epsilon)
    : LinearJointInterface(lowerBound, upperBound, epsilon),
      tap::control::Subsystem(drivers),
      pid(config),
      motor(motor),
      kS(kS)
{
    this->setpoint = 0;
}

void JointSubsystem::initialize() { motor.initialize(); }

float JointSubsystem::getPosition()
{
    return motor.getEncoder()->getPosition().getUnwrappedValue();
}

void JointSubsystem::refresh()
{
    float error = setpoint - getPosition();
    float output = pid.runController(error, motor.getEncoder()->getVelocity(), 2.0f) + kS;
    motor.setDesiredOutput(output);
}

void JointSubsystem::refreshSafeDisconnect() { motor.setDesiredOutput(0); }

}  // namespace engineer
}  // namespace aruwsrc