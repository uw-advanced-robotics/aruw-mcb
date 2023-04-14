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
    const tap::algorithms::SmoothPidConfig fivebarMotor1PidConfig,
    const tap::algorithms::SmoothPidConfig fivebarMotor2PidConfig,
    tap::motor::MotorInterface* wheelMotor,
    const float wheelRadius,
    const tap::algorithms::SmoothPidConfig driveWheelPidConfig)
    : fivebar(fivebar),
      fiveBarMotor1Pid(fivebarMotor1PidConfig),
      fiveBarMotor2Pid(fivebarMotor2PidConfig),
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
    modm::Vector2f desiredWheelLocation = modm::Vector2f(0, 0.125);
    float desiredWheelSpeed = vDesired;
    float desiredWheelAngle = 0;
    // desiredWheelSpeed += 1 / WHEEL_RADIUS * (vDesired - zCurrent * tl_dot);
    desiredWheelAngle -=
        fivebar->getCurrentPosition().getOrientation() - motorLinkAnglePrev;  // subtract
    motorLinkAnglePrev = fivebar->getCurrentPosition().getOrientation();

    xoffset = xPid.runControllerDerivateError(vDesired - vCurrent, dt);

    float desiredx = cos(chassisAngle) * xoffset + sin(chassisAngle) * zDesired;
    float desiredz = -sin(chassisAngle) * xoffset + cos(chassisAngle) * zDesired;

    desiredWheelLocation.setX(tap::algorithms::limitVal(desiredx, -.1f, .1f));
    desiredWheelLocation.setY(tap::algorithms::limitVal(desiredz, -.35f, -.15f));

    desiredWheelSpeed -= thetaLPid.runControllerDerivateError(-(tl + chassisAngle), dt);

    float driveWheelSpeedError = desiredWheelSpeed - WHEEL_RADIUS * realWheelSpeed;

    float driveWheelOutput = driveWheelPid.runControllerDerivateError(driveWheelSpeedError, dt);

    driveWheelOutput += desiredWheelAngle * 1000 / dt;  // add to rad/s, rad to move within a dt

    // 3. Send New Output Values
    driveWheel->setDesiredOutput(driveWheelOutput);
    fivebar->setDesiredPosition(desiredWheelLocation);
    fivebar->refresh();
    fiveBarController(dt);
}

void BalancingLeg::fiveBarController(uint32_t dt)
{
    float L = fivebar->getFiveBarConfig().motor1toMotor2Length;
    float gravT1 = fivebar->getFiveBarConfig().motor1toJoint1Length *
                   cos(fivebar->getMotor1RelativePosition()) *
                   ((fivebar->getCurrentPosition().getX() + (L / 2)) / L) *
                   (MASS_CHASSIS * 9.81 / 2);
    float gravT2 = fivebar->getFiveBarConfig().motor2toJoint2Length *
                   cos(M_PI - fivebar->getMotor2RelativePosition()) *
                   (-(fivebar->getCurrentPosition().getX() - (L / 2)) / L) *
                   (MASS_CHASSIS * 9.81 / 2);

    float motor1error = fivebar->getMotor1Error();
    float motor2error = fivebar->getMotor2Error();

    float motor1output = fiveBarMotor1Pid.runController(
        motor1error,
        fivebar->getMotor1()->getShaftRPM() * M_TWOPI / 60,
        dt);
    float motor2output = fiveBarMotor2Pid.runController(
        motor2error,
        fivebar->getMotor2()->getShaftRPM() * M_TWOPI / 60,
        dt);

    motor1output -= 1000 * gravT1 / aruwsrc::motor::AK809_TORQUE_CONSTANT;
    // motor direction so minus
    motor2output += 1000 * gravT2 / aruwsrc::motor::AK809_TORQUE_CONSTANT;

    fivebar->moveMotors(motor1output, motor2output);
}

void BalancingLeg::computeState(uint32_t dt)
{
    zCurrent = fivebar->getCurrentPosition().getX() * sin(chassisAngle) +
               fivebar->getCurrentPosition().getY() * cos(chassisAngle);
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

    float chassisAngledotNew = (chassisAngle - chassisAnglePrev) * 1000 / dt;

    chassisAngledot = tap::algorithms::lowPassFilter(chassisAngledot, chassisAngledotNew, .5);
    // chassisAngledot = tap::algorithms::lowPassFilter(chassisAngledot, chassisAngledotNew, .1);
}
}  // namespace chassis
}  // namespace aruwsrc
