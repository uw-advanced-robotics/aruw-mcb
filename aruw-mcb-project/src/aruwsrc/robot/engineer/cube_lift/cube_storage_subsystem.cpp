/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "cube_storage_subsystem.hpp"

#include "tap/motor/dji_motor.hpp"

namespace aruwsrc::engineer::lift
{
CubeStorageSubsystem::CubeStorageSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface& storageLiftMotor,
    const tap::algorithms::SmoothPidConfig& configPos,
    aruwsrc::control::TriggerInterface& trigger,
    const LimitSwitchSetpointInterface::LimitSwitchConfig& limitConfig)
    : LimitSwitchSetpointInterface(
          drivers,
          trigger,
          configPos,
          limitConfig),
      motor(storageLiftMotor)
{
    calibrationState = CalibrationState::AWAITING_CALIBRATE;
};

void CubeStorageSubsystem::initialize()
{
    motor.initialize();
    setDesiredOutput(0);
}

void CubeStorageSubsystem::setDesiredOutput(int16_t power) { motor.setDesiredOutput(power); }

void CubeStorageSubsystem::resetEncoderValue() { motor.getEncoder()->resetEncoderValue(); }

float CubeStorageSubsystem::getEncoderValue()
{
    return motor.getEncoder()->getPosition().getUnwrappedValue();
}

float CubeStorageSubsystem::getEncoderVelocity() { return motor.getEncoder()->getVelocity(); }

// void CubeStorageSubsystem::setVelocitySetpoint(float newSetpoint)
// {
//     velocitySetpoint = newSetpoint;
// }

// float CubeStorageSubsystem::getVelocitySetpoint() { return velocitySetpoint; }
/*potentially use if whicher controls lead/aiden wants us to (we forgot which)*/

}  // namespace aruwsrc::engineer::lift
