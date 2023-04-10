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

#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"

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
    zDesired = fivebar->getDefaultPosition().getY();
}

void BalancingLeg::update()
{
    // 1. Update Current State

    uint32_t currentTime = tap::arch::clock::getTimeMilliseconds();
    int32_t dt = currentTime - prevTime;
    prevTime = currentTime;
    computeState(dt);

    // 2. Apply Control Law
    modm::Vector2f desiredWheelLocation = modm::Vector2f(0, zDesired);
    float desiredWheelSpeed = 0;
    float desiredWheelAngle = 0;
    // desiredWheelSpeed += 1 / WHEEL_RADIUS * (vDesired - zCurrent * tl_dot);
    desiredWheelSpeed += vDesired / WHEEL_RADIUS;
    desiredWheelAngle -=
        fivebar->getCurrentPosition().getOrientation() - motorLinkAnglePrev;  // subtract
    motorLinkAnglePrev = fivebar->getCurrentPosition().getOrientation();

    xoffset = xPid.runControllerDerivateError(aDesired - aCurrent, dt);

    // xoffset = tap::algorithms::limitVal<float>(0.0005 * aDesired / (9.81 * WHEEL_RADIUS), -.03,
    // .03); xoffsetPrev = tap::algorithms::lowPassFilter(xoffsetPrev, xoffset, .05); xoffset =
    // tap::algorithms::lowPassFilter(xoffsetPrev, xoffset, .05);
    // desiredWheelLocation.setX(xoffset);

    float driveWheelOutput =
        driveWheelPid.runControllerDerivateError(desiredWheelSpeed - realWheelSpeed, dt);
    driveWheelOutput += desiredWheelAngle * 1000 / dt;  // add to rad/s, rad to move within a dt

    // 3. Send New Output Values
    driveWheel->setDesiredOutput(driveWheelOutput);
    fivebar->setDesiredPosition(desiredWheelLocation);
    fivebar->refresh();
}

void BalancingLeg::computeState(uint32_t dt)
{
    zCurrent = fivebar->getCurrentPosition().getY();
    tl = atan2(-fivebar->getCurrentPosition().getX(), -zCurrent);
    tl_dot = 1000 * (tl - tl_prev) / dt;  // ms to s
    tl_prev = tl;
    tl_dot = tap::algorithms::lowPassFilter(tl_dotPrev, tl_dot, .2);
    tl_dotPrev = tl_dot;

    float wheelPos = 0;
    wheelPos = driveWheel->getPositionUnwrapped() * CHASSIS_GEARBOX_RATIO;  // rad
    realWheelSpeed = 1000 * (wheelPos - wheelPosPrev) / dt;                 // rad/s
    wheelPosPrev = wheelPos;

    vCurrent = realWheelSpeed * WHEEL_RADIUS + zCurrent * tl_dot;  // m/s
    aCurrent = (vCurrent - vCurrentPrev) * 1000 / dt;
    vCurrentPrev = vCurrent;
    aCurrentPrev = tap::algorithms::lowPassFilter(aCurrentPrev, aCurrent, .1);
    aCurrent = tap::algorithms::lowPassFilter(aCurrentPrev, aCurrent, .1);

    aDesired = (vDesired - vDesiredPrev) * 1000 / dt;
    aDesiredPrev = tap::algorithms::lowPassFilter(aDesiredPrev, aDesired, .1);
    aDesired = tap::algorithms::lowPassFilter(aDesiredPrev, aDesired, .1);
}
}  // namespace chassis
}  // namespace aruwsrc
