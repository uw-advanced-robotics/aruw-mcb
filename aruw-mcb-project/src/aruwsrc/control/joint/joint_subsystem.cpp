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

#include "aruwsrc/control/joint/joint_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

namespace aruwsrc::control::joint
{
JointSubsystem::JointSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface& motor,
    Config config)
    : tap::control::Subsystem(drivers),
      motor(motor),
      epsilon(config.epsilon),
      maxSetpointIncrement(config.maxSetpointIncrement),
      posPid(config.posPidConfig),
      encoderRatio(config.encoderRatio),
      staticFeedforward(config.staticFeedforward),
      maxOutput(config.maxOutput)
{
}

void JointSubsystem::initialize() { motor.initialize(); }

float JointSubsystem::getPosition() const
{
    return motor.getEncoder()->getPosition().getUnwrappedValue() * encoderRatio;
}

float JointSubsystem::getVelocity() const
{
    return motor.getEncoder()->getVelocity() * encoderRatio;
}

void JointSubsystem::setLowerBound(float lowerBound)
{
    if (lowerBound > this->upperBound) return;
    this->lowerBound = lowerBound;
}

void JointSubsystem::setUpperBound(float upperBound)
{
    if (upperBound < this->lowerBound) return;
    this->upperBound = upperBound;
}

void JointSubsystem::setSetpoint(float setpoint)
{
    if (tap::algorithms::compareFloatClose(lowerBound, upperBound, epsilon))
        this->setpoint.setTarget(setpoint);
    else
        this->setpoint.setTarget(std::clamp(setpoint, lowerBound, upperBound));
};

bool JointSubsystem::atSetpoint()
{
    return tap::algorithms::compareFloatClose(setpoint.getTarget(), getPosition(), epsilon);
};

bool JointSubsystem::isOnline() const { return motor.isMotorOnline(); }

void JointSubsystem::runPosPidController(float dt)
{
    error = setpoint.getValue() - getPosition();
    motorDesiredOutput = posPid.runController(error, getVelocity(), dt) + staticFeedforward;
    motor.setDesiredOutput(std::clamp(motorDesiredOutput, -maxOutput, maxOutput));
}

void JointSubsystem::refresh()
{
    debug_position = getPosition();
    this->updateSetpoint();
    runPosPidController(2.0f);  // todo: should be 0.002 but would requires retune
}

void JointSubsystem::refreshSafeDisconnect() { motor.setDesiredOutput(0); }

}  // namespace aruwsrc::control::joint