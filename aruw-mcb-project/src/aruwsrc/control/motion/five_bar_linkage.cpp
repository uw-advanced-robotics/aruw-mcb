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

#include "five_bar_linkage.hpp"

#include <cassert>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"

#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"

namespace aruwsrc::control::motion
{
FiveBarLinkage::FiveBarLinkage(
    tap::Drivers* drivers,
    tap::motor::MotorInterface* motor1,
    tap::motor::MotorInterface* motor2,
    FiveBarConfig fiveBarConfig,
    tap::algorithms::SmoothPidConfig motorPidConfig)
    : motor1Pid(motorPidConfig),
      motor2Pid(motorPidConfig),
      fiveBarConfig(fiveBarConfig)
{
    assert(drivers != nullptr);
    assert(motor1 != nullptr);
    assert(motor2 != nullptr);
}

void FiveBarLinkage::initialize() { desiredPosition = fiveBarConfig.defaultPosition; }

void FiveBarLinkage::refresh()
{
    computeMotorAngles();
    moveMotors();
}

void FiveBarLinkage::moveMotors()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    float motor1Output =
        motor1Pid.runControllerDerivateError(motor1->getPositionUnwrapped() - motor1Setpoint, dt);
    motor1->setDesiredOutput(motor1Output);
    float motor2Output =
        motor2Pid.runControllerDerivateError(motor2->getPositionUnwrapped() - motor1Setpoint, dt);
    motor2->setDesiredOutput(motor2Output);
}

void FiveBarLinkage::computeMotorAngles()
{
    float xp = desiredPosition.getX() +
               fiveBarConfig.motor1toMotor2Length / 2;  // uncenter the computation point
    float c1Inv = tap::algorithms::fastInvSqrt(powf(xp, 2) + powf(desiredPosition.getY(), 2));
    float c2Inv = tap::algorithms::fastInvSqrt(
        powf(xp, 2) + powf(desiredPosition.getY(), 2) -
        2 * xp * fiveBarConfig.motor1toMotor2Length + powf(fiveBarConfig.motor1toMotor2Length, 2));

    this->motor1Setpoint = M_PI + acos(xp * c1Inv) +
                           acos(
                               ((powf(fiveBarConfig.joint1toTipLength, 2) -
                                 powf(fiveBarConfig.motor1toJoint1Length, 2) - powf(xp, 2) +
                                 powf(desiredPosition.getY(), 2)) *
                                c1Inv) /
                               (-2 * fiveBarConfig.motor1toJoint1Length));
    this->motor2Setpoint = 2 * M_PI - acos((-xp + fiveBarConfig.motor1toMotor2Length) * c2Inv) -
                           acos(
                               ((powf(fiveBarConfig.joint2toTipLength, 2) -
                                 powf(fiveBarConfig.motor2toJoint2Length, 2) -
                                 (powf(xp, 2) + powf(desiredPosition.getY(), 2) -
                                  2 * xp * fiveBarConfig.motor1toMotor2Length +
                                  powf(fiveBarConfig.motor1toMotor2Length, 2))) *
                                c2Inv) /
                               (-2 * fiveBarConfig.motor2toJoint2Length));
}

}  // namespace aruwsrc::control::motion
