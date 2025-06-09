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

#include "aruwsrc/robot/engineer/gantry/gantry_extension_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

namespace aruwsrc::engineer::gantry
{
GantryExtensionSubsystem::GantryExtensionSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface& motor,
    const tap::algorithms::SmoothPidConfig& config,
    control::TriggerInterface& trigger,
    float radius,
    float lowerBound,
    float upperBound,
    float home,
    float kS,
    float epsilon)
    : LimitSwitchSetpointInterface(
          drivers,
          trigger,
          config,
          radius,
          lowerBound,
          upperBound,
          home,
          kS,
          epsilon,
          10.0f,
          false),
      motor(motor)
{
}

void GantryExtensionSubsystem::initialize()
{
    motor.initialize();
    motor.getEncoder()->resetEncoderValue();
}

void GantryExtensionSubsystem::setDesiredOutput(int16_t power) { motor.setDesiredOutput(power); }

void GantryExtensionSubsystem::resetEncoderValue() { motor.getEncoder()->resetEncoderValue(); }

float GantryExtensionSubsystem::getEncoderValue()
{
    return motor.getEncoder()->getPosition().getUnwrappedValue();
}

float GantryExtensionSubsystem::getEncoderVelocity() { return motor.getEncoder()->getVelocity(); }

}  // namespace aruwsrc::engineer::gantry