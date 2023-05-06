/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "barrel_switcher_subsystem.hpp"

#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"

namespace aruwsrc::control
{
BarrelSwitcherSubsystem::BarrelSwitcherSubsystem(
    tap::Drivers* drivers,
    aruwsrc::control::HomingConfig config,
    tap::motor::MotorId motorid)
    : HomeableSubsystemInterface(drivers),
      encoderPid(
          POSITION_PID_KP,
          POSITION_PID_KI,
          POSITION_PID_KD,
          POSITION_PID_MAX_ERROR_SUM,
          POSITION_PID_MAX_OUTPUT),
      config(config),
      motor(drivers, motorid, tap::can::CanBus::CAN_BUS1, false, "barrel switching motor")
{
}

void BarrelSwitcherSubsystem::initialize()
{
    barrelState = BarrelState::BETWEEN_BARRELS;
    motor.initialize();
}

void BarrelSwitcherSubsystem::refresh()
{
    outputDesiredDebug = motor.getOutputDesired();
    torqueDebug = motor.getTorque();
    shaftRPMDebug = motor.getShaftRPM();
    stalled = this->isStalled();
    switch (barrelState)
    {
        case BarrelState::MOVING_TOWARD_LOWER_BOUND:
            setMotorOutput(-HOMING_MOTOR_OUTPUT);
            if(this->isHomed() && motor.getEncoderUnwrapped() <= 0)
            {
                this->barrelState = BarrelState::USING_LEFT_BARREL;
            }
            break;
        case BarrelState::MOVING_TOWARD_UPPER_BOUND:
            setMotorOutput(HOMING_MOTOR_OUTPUT);
            if(this->isHomed() && motor.getEncoderUnwrapped() >= 0)
            {
                this->barrelState = BarrelState::USING_RIGHT_BARREL;
            }
            break;
        case BarrelState::USING_LEFT_BARREL:
            updateMotorEncoderPid(&encoderPid, &motor, 0);
            break;
        case BarrelState::USING_RIGHT_BARREL:
            updateMotorEncoderPid(&encoderPid, &motor, motorUpperBound);
            break;
        case BarrelState::BETWEEN_BARRELS:
            break;
    }
}

BarrelState BarrelSwitcherSubsystem::getBarrelState() { return barrelState; }

void BarrelSwitcherSubsystem::setMotorOutput(int32_t desiredOutput)
{
    if (lowerBoundSet && upperBoundSet &&
        ((motor.getEncoderUnwrapped() <= 0 && desiredOutput < 0) ||
         (motor.getEncoderUnwrapped() >= motorUpperBound && desiredOutput > 0)))
    {
        desiredOutput = 0;
    }
    motor.setDesiredOutput(desiredOutput);
}

bool BarrelSwitcherSubsystem::isStalled() const
{
    return (
        (motor.getShaftRPM() > config.minRPM && motor.getShaftRPM() < config.maxRPM) &&
        (motor.getTorque() > config.maxTorque || motor.getTorque() < config.minTorque));
}

void BarrelSwitcherSubsystem::setLowerBound()
{
    motor.resetEncoderValue();
    lowerBoundSet = true;
}

void BarrelSwitcherSubsystem::setUpperBound()
{
    motorUpperBound = motor.getEncoderUnwrapped();
    upperBoundSet = true;
}

bool BarrelSwitcherSubsystem::isHomed() {
    return upperBoundSet && lowerBoundSet;
}

void BarrelSwitcherSubsystem::moveTowardUpperBound()
{
    barrelState = BarrelState::MOVING_TOWARD_UPPER_BOUND;
}

void BarrelSwitcherSubsystem::moveTowardLowerBound()
{
    barrelState = BarrelState::MOVING_TOWARD_LOWER_BOUND;
}

void BarrelSwitcherSubsystem::stop()
{
    this->setMotorOutput(0);
    barrelState = BarrelState::USING_RIGHT_BARREL;
}

void BarrelSwitcherSubsystem::updateMotorEncoderPid(
    modm::Pid<int32_t>* pid,
    tap::motor::DjiMotor* const motor,
    int32_t desiredEncoderPosition)
{
    pid->update(desiredEncoderPosition - motor->getEncoderUnwrapped());
    setMotorOutput(pid->getValue());
}

};  // namespace aruwsrc::control