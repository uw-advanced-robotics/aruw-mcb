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

#include "friction_wheel_subsystem.hpp"

#include "tap/architecture/clock.hpp"

#include "aruwsrc/drivers.hpp"

namespace aruwsrc
{
namespace launcher
{
FrictionWheelSubsystem::FrictionWheelSubsystem(
    aruwsrc::Drivers *drivers,
    tap::motor::MotorId leftMotorId,
    tap::motor::MotorId rightMotorId)
    : tap::control::Subsystem(drivers),
      velocityPidLeftWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
      velocityPidRightWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
      desiredRpmRamp(0),
      leftWheel(drivers, leftMotorId, CAN_BUS_MOTORS, true, "left example motor"),
      rightWheel(drivers, rightMotorId, CAN_BUS_MOTORS, false, "right example motor")
{
}

void FrictionWheelSubsystem::initialize()
{
    leftWheel.initialize();
    rightWheel.initialize();
}

void FrictionWheelSubsystem::setDesiredRpm(float desRpm) { desiredRpmRamp.setTarget(desRpm); }

void FrictionWheelSubsystem::refresh()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    desiredRpmRamp.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));
    prevTime = currTime;

    velocityPidLeftWheel.update(desiredRpmRamp.getValue() - leftWheel.getShaftRPM());
    leftWheel.setDesiredOutput(static_cast<int32_t>(velocityPidLeftWheel.getValue()));
    velocityPidRightWheel.update(desiredRpmRamp.getValue() - rightWheel.getShaftRPM());
    rightWheel.setDesiredOutput(static_cast<int32_t>(velocityPidRightWheel.getValue()));
}

void FrictionWheelSubsystem::runHardwareTests()
{
    if (abs(rightWheel.getShaftRPM()) > 4000.0f) this->setHardwareTestsComplete();
}

void FrictionWheelSubsystem::onHardwareTestStart() { this->setDesiredRpm(5000.0f); }

void FrictionWheelSubsystem::onHardwareTestComplete() { this->setDesiredRpm(0.0f); }

}  // namespace launcher

}  // namespace aruwsrc
