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

#include "aruwsrc/robot/engineer/joint_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

namespace aruwsrc::engineer
{
JointSubsystem::JointSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface& motor,
    Config config)
    : tap::control::Subsystem(drivers),
      LinearJointInterface(config.super),
      motor(motor),
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

void JointSubsystem::refresh()
{
    this->updateSetpoint();
    runPosPidController(2.0f);  // todo: should be 0.002 but would requires retune
}

void JointSubsystem::refreshSafeDisconnect() { motor.setDesiredOutput(0); }

}  // namespace aruwsrc::engineer