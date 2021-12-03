/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "example_subsystem.hpp"

#include "aruwsrc/drivers.hpp"

namespace aruwsrc
{
namespace control
{
const tap::motor::MotorId ExampleSubsystem::LEFT_MOTOR_ID = tap::motor::MOTOR2;
const tap::motor::MotorId ExampleSubsystem::RIGHT_MOTOR_ID = tap::motor::MOTOR1;

ExampleSubsystem::ExampleSubsystem(
    aruwsrc::Drivers* drivers,
    tap::motor::MotorId leftMotorId,
    tap::motor::MotorId rightMotorId)
    : tap::control::Subsystem(drivers),
      velocityPidLeftWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
      velocityPidRightWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
      desiredRpm(0),
      leftWheel(drivers, leftMotorId, CAN_BUS_MOTORS, true, "left example motor"),
      rightWheel(drivers, rightMotorId, CAN_BUS_MOTORS, false, "right example motor")
{
}

void ExampleSubsystem::initialize()
{
    leftWheel.initialize();
    rightWheel.initialize();
}

void ExampleSubsystem::setDesiredRpm(float desRpm) { desiredRpm = desRpm; }

void ExampleSubsystem::refresh()
{
    updateMotorRpmPid(&velocityPidLeftWheel, &leftWheel, desiredRpm);
    updateMotorRpmPid(&velocityPidRightWheel, &rightWheel, desiredRpm);
}

void ExampleSubsystem::updateMotorRpmPid(
    modm::Pid<float>* pid,
    tap::motor::DjiMotor* motor,
    float desiredRpm)
{
    pid->update(desiredRpm - motor->getShaftRPM());
    motor->setDesiredOutput(static_cast<int32_t>(pid->getValue()));
}

void ExampleSubsystem::runHardwareTests()
{
    // TODO
}

}  // namespace control

}  // namespace aruwsrc
