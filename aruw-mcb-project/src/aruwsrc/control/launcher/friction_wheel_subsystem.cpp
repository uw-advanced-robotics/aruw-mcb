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

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"

#include "aruwsrc/drivers.hpp"

using namespace tap::algorithms;

namespace aruwsrc::control::launcher
{
FrictionWheelSubsystem::FrictionWheelSubsystem(
    aruwsrc::Drivers *drivers,
    tap::motor::MotorId leftMotorId,
    tap::motor::MotorId rightMotorId)
    : tap::control::Subsystem(drivers),
      launchSpeedLinearInterpolator(
          LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT,
          MODM_ARRAY_SIZE(LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT)),
      velocityPidLeftWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
      velocityPidRightWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
      desiredRpmRamp(0),
      leftWheel(drivers, leftMotorId, CAN_BUS_MOTORS, true, "Left flywheel"),
      rightWheel(drivers, rightMotorId, CAN_BUS_MOTORS, false, "Right flywheel")
{
}

void FrictionWheelSubsystem::initialize()
{
    leftWheel.initialize();
    rightWheel.initialize();
}

void FrictionWheelSubsystem::setDesiredLaunchSpeed(float speed)
{
    desiredLaunchSpeed = speed;
    desiredRpmRamp.setTarget(launchSpeedToFrictionWheelRpm(speed));
}

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

void FrictionWheelSubsystem::onHardwareTestStart() { this->setDesiredLaunchSpeed(15); }

void FrictionWheelSubsystem::onHardwareTestComplete() { this->setDesiredLaunchSpeed(0); }

float FrictionWheelSubsystem::launchSpeedToFrictionWheelRpm(float launchSpeed) const
{
    launchSpeed = limitVal(launchSpeed, 0.0f, LAUNCH_SPEED_MAX);
    return launchSpeedLinearInterpolator.interpolate(launchSpeed);
}

}  // namespace aruwsrc::control::launcher
