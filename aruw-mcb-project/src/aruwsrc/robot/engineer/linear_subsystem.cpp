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

#include "linear_subsystem.hpp"

namespace aruwsrc::engineer
{
LinearSubsystem::LinearSubsystem(
    tap::Drivers* drivers,
    const LinearSubsystemConfig& config,
    tap::motor::MotorInterface* motor)
    : tap::control::Subsystem(drivers),
      motor(motor),
      positionPid(config.p, config.i, config.d, config.maxErrorSum, config.maxOutput),
      setpointTolerance(config.setpointTolerance),
      feedforward(config.feedforward),
      setpointToEncoderScalar(config.setpointToEncoderScalar)
{
}

void LinearSubsystem::initialize()
{
    motor->initialize();
    positionPid.reset();
}

void LinearSubsystem::refresh()
{
    positionPid.update(setpoint - motor->getEncoderUnwrapped());
    motor->setDesiredOutput(positionPid.getValue() + feedforward);
}

void LinearSubsystem::refreshSafeDisconnect() { motor->setDesiredOutput(0); }

void LinearSubsystem::setSetpoint(float setpoint)
{
    this->setpoint = setpoint * setpointToEncoderScalar;
}

float LinearSubsystem::getPosition()
{
    return motor->getEncoderUnwrapped() / setpointToEncoderScalar;
}

}  // namespace aruwsrc::engineer
