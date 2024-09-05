/*
 * Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "joint_subsystem.hpp"

namespace aruwsrc::engineer
{
JointSubsystem::JointSubsystem(
    tap::Drivers* drivers,
    const JointSubsystemConfig& config,
    tap::motor::MotorInterface* motor,
    const char* jointName)
    : tap::control::Subsystem(drivers),
      motor(motor),
      positionPid(config.p, config.i, config.d, config.maxErrorSum, config.maxOutput),
      setpointTolerance(config.setpointTolerance),
      feedforward(config.feedforward),
      setpointToEncoderScalar(config.setpointToEncoderScalar),
      name(jointName)
{
}

void JointSubsystem::initialize()
{
    motor->initialize();
    positionPid.reset();
}

void JointSubsystem::refresh()
{
    positionPid.update(setpoint - motor->getEncoderUnwrapped());
    motor->setDesiredOutput(positionPid.getValue() + feedforward);
}

void JointSubsystem::refreshSafeDisconnect() { motor->setDesiredOutput(0); }

void JointSubsystem::setSetpoint(float setpoint)
{
    this->setpoint = setpoint * setpointToEncoderScalar;
}

float JointSubsystem::getSetpoint() { return setpoint / setpointToEncoderScalar; }

float JointSubsystem::getPosition()
{
    return motor->getEncoderUnwrapped() / setpointToEncoderScalar;
}

}  // namespace aruwsrc::engineer
