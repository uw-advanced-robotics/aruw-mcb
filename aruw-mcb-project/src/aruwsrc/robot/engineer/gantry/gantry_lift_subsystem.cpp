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

#include "aruwsrc/robot/engineer/gantry/gantry_lift_subsystem.hpp"
namespace aruwsrc::engineer::gantry
{
GantryLiftSubsystem::GantryLiftSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface& motorLeft,
    tap::motor::MotorInterface& motorRight,
    const tap::algorithms::SmoothPidConfig& configPos,
    const tap::algorithms::SmoothPidConfig& configAlign,
    control::TriggerInterface& trigger,
    const LimitSwitchSetpointInterface::LimitSwitchConfig &limitConfig)
    : LimitSwitchSetpointInterface(
          drivers,
          trigger,
          configPos,
          limitConfig),
      pidAlign(configAlign),
      motorLeft(motorLeft),
      motorRight(motorRight)
{
}

void GantryLiftSubsystem::initialize()
{
    motorLeft.initialize();
    motorRight.initialize();

    motorLeft.getEncoder()->resetEncoderValue();
    motorRight.getEncoder()->resetEncoderValue();
}

void GantryLiftSubsystem::setDesiredOutput(int16_t output)
{
    float errorAlignment = getPositionDifference();

    float outputAlign = pidAlign.runController(errorAlignment, getVelocityDifference(), 2.0f);

    motorLeft.setDesiredOutput(output + outputAlign + limitSwitchConfig.kS);
    motorRight.setDesiredOutput(output - outputAlign + limitSwitchConfig.kS);
}

void GantryLiftSubsystem::resetEncoderValue()
{
    motorLeft.getEncoder()->resetEncoderValue();
    motorRight.getEncoder()->resetEncoderValue();
}

float GantryLiftSubsystem::getEncoderValue()
{
    return (motorLeft.getEncoder()->getPosition().getUnwrappedValue() +
            motorRight.getEncoder()->getPosition().getUnwrappedValue()) /
           2;
}

float GantryLiftSubsystem::getEncoderVelocity()
{
    return (motorLeft.getEncoder()->getVelocity() + motorRight.getEncoder()->getVelocity()) / 2;
}

float GantryLiftSubsystem::getPositionDifference()
{
    return (motorLeft.getEncoder()->getPosition().getUnwrappedValue() -
            motorRight.getEncoder()->getPosition().getUnwrappedValue()) *
           limitSwitchConfig.radius;
}

float GantryLiftSubsystem::getVelocityDifference()
{
    return (motorLeft.getEncoder()->getVelocity() - motorRight.getEncoder()->getVelocity()) *
           limitSwitchConfig.radius;
}

void GantryLiftSubsystem::stopDuringHoming() { refreshSafeDisconnect(); }

void GantryLiftSubsystem::refreshSafeDisconnect()
{
    motorLeft.setDesiredOutput(0);
    motorRight.setDesiredOutput(0);
}

}  // namespace aruwsrc::engineer::gantry