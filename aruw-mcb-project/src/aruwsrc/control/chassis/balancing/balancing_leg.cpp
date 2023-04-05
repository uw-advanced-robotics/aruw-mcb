/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "balancing_leg.hpp"

#include <assert.h>

namespace aruwsrc
{
namespace chassis
{
BalancingLeg::BalancingLeg(
    tap::Drivers* drivers,
    aruwsrc::control::motion::FiveBarLinkage* fivebar,
    tap::motor::MotorInterface* wheelMotor,
    const float wheelRadius,
    const tap::algorithms::SmoothPidConfig driveWheelPidConfig)
    : fivebar(fivebar),
      driveWheel(wheelMotor),
      WHEEL_RADIUS(wheelRadius),
      driveWheelPid(driveWheelPidConfig)
{
    assert(fivebar != nullptr);
    assert(wheelMotor != nullptr);
}

void BalancingLeg::initialize()
{
    driveWheel->initialize();
    fivebar->initialize();
}

void BalancingLeg::update()
{
    // 1. Update Current State
    uint32_t currentTime = tap::arch::clock::getTimeMilliseconds();
    int32_t dt = currentTime - prevTime;
    prevTime = currentTime;
    zCurrent = fivebar->getCurrentPosition().getY();
    tl_Current = atan2(zCurrent, -fivebar->getCurrentPosition().getX());
    vCurrent = WHEEL_RADIUS * driveWheel->getShaftRPM() * M_TWOPI / 60;

    // 2. Apply Control Law
    modm::Vector2f desiredWheelLocation = modm::Vector2f(0, -.2);
    float desiredWheelSpeed = 0;
    float desiredWheelAngle = 0;
    desiredWheelSpeed += vDesired;
    desiredWheelAngle -=
        fivebar->getCurrentPosition().getOrientation() - motorLinkAnglePrev;  // subtract
    motorLinkAnglePrev = fivebar->getCurrentPosition().getOrientation();

    // 3. Send New Output Values
    fivebar->setDesiredPosition(desiredWheelLocation);
    // we do things in mks here
    float driveWheelOutput = driveWheelPid.runControllerDerivateError(
        driveWheel->getShaftRPM() * M_TWOPI / 360 - desiredWheelSpeed,
        dt);

    driveWheel->setDesiredOutput(driveWheelOutput);
    fivebar->refresh();
}
}  // namespace chassis
}  // namespace aruwsrc
