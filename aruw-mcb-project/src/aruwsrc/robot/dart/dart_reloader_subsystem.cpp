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

#include "aruwsrc/robot/dart/dart_reloader_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "dart_constants.hpp"
namespace aruwsrc::dart
{
DartReloaderSubsystem::DartReloaderSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface& motor)
    : Subsystem(drivers),
      motor(motor),
      pidController(DART_RELOADER_PID_CONFIG)
{
}
void DartReloaderSubsystem::initialize()
{
    motor.initialize();
    pidController.reset();
}
void DartReloaderSubsystem::setSetpoint(float32_t setpoint) { this->setpoint = setpoint; }

bool DartReloaderSubsystem::atSetpoint()
{
    float error = motor.getEncoder()->getPosition().getUnwrappedValue() - setpoint;
    return tap::algorithms::compareFloatClose(error, 0.0f, DART_RELOADER_PID_CONFIG.errDeadzone);
}
void DartReloaderSubsystem::refresh()
{
    float currentPosition = motor.getEncoder()->getPosition().getUnwrappedValue();
    float error = setpoint - currentPosition;
    float errorDerivative = -motor.getEncoder()->getVelocity();
    float output = pidController.runController(error, errorDerivative, 0.002f);
    motor.setDesiredOutput(output);
    position = currentPosition;
}

void DartReloaderSubsystem::refreshSafeDisconnect() { motor.setDesiredOutput(0); }

}  // namespace aruwsrc::robot::dart