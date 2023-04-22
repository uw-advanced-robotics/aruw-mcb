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
    tap::motor::MotorInterface* motor1,
    tap::motor::MotorInterface* motor2,
    FiveBarConfig fiveBarConfig)
    : motor1(motor1),
      motor2(motor2),
      fiveBarConfig(fiveBarConfig)
{
    assert(motor1 != nullptr);
    assert(motor2 != nullptr);
}

void FiveBarLinkage::initialize()
{
    desiredPosition = fiveBarConfig.defaultPosition;
    computeMotorAngles();
    motor1home = motor1Setpoint;  // Zeroes the motors, assuming you boot at default postion
    motor2home = motor2Setpoint;

    motor1->initialize();
    motor1->setDesiredOutput(0);
    motor2->initialize();
    motor2->setDesiredOutput(0);
}

void FiveBarLinkage::refresh()
{
    motor1RelativePosition = motor1->getPositionUnwrapped() + motor1home;
    motor2RelativePosition = motor2->getPositionUnwrapped() + motor2home;

    motor1->isMotorOnline();
    motor2->isMotorOnline();

    computeMotorAngles();
    computePositionFromAngles();
}

float FiveBarLinkage::getMotor1Error()
{
    return tap::algorithms::limitVal(
               motor1Setpoint,
               fiveBarConfig.motor1MinAngle,
               fiveBarConfig.motor1MaxAngle) -
           motor1RelativePosition;
};
float FiveBarLinkage::getMotor2Error()
{
    return tap::algorithms::limitVal(
               motor2Setpoint,
               fiveBarConfig.motor1MinAngle,
               fiveBarConfig.motor2MaxAngle) -
           motor2RelativePosition;
};

void FiveBarLinkage::moveMotors(float motor1output, float motor2output)
{
    motor1->setDesiredOutput(motor1output);
    motor2->setDesiredOutput(motor2output);
}

void FiveBarLinkage::computeMotorAngles()
{
    float xp = desiredPosition.getX() +
               fiveBarConfig.motor1toMotor2Length / 2;  // uncenter the computation point
    float c1Inv = tap::algorithms::fastInvSqrt(powf(xp, 2) + powf(desiredPosition.getY(), 2));
    float c1Squared = powf(xp, 2) + powf(desiredPosition.getY(), 2);
    float c2Inv = tap::algorithms::fastInvSqrt(
        powf(xp, 2) + powf(desiredPosition.getY(), 2) -
        2 * xp * fiveBarConfig.motor1toMotor2Length + powf(fiveBarConfig.motor1toMotor2Length, 2));
    float c2Squared = powf(xp, 2) + powf(desiredPosition.getY(), 2) -
                      2 * xp * fiveBarConfig.motor1toMotor2Length +
                      powf(fiveBarConfig.motor1toMotor2Length, 2);

    motor1Setpoint = M_PI + acosf(xp * c1Inv) +
                     acosf(
                         ((powf(fiveBarConfig.joint1toTipLength, 2) -
                           powf(fiveBarConfig.motor1toJoint1Length, 2) - c1Squared) *
                          c1Inv) /
                         (-2 * fiveBarConfig.motor1toJoint1Length));
    motor2Setpoint = 2 * M_PI - acosf((-xp + fiveBarConfig.motor1toMotor2Length) * c2Inv) -
                     acosf(
                         ((powf(fiveBarConfig.joint2toTipLength, 2) -
                           powf(fiveBarConfig.motor2toJoint2Length, 2) - c2Squared) *
                          c2Inv) /
                         (-2 * fiveBarConfig.motor2toJoint2Length));
    return;
}

bool FiveBarLinkage::withinEnvelope(modm::Vector2f point)
{
    // TODO: this
    return true;
}
void FiveBarLinkage::computePositionFromAngles()
{
    float currentX = tap::algorithms::interpolateLinear2D(
                         chassis::FIVE_BAR_LUT_X,
                         chassis::FIVE_BAR_T1_MIN,
                         chassis::FIVE_BAR_T1_MAX,
                         chassis::FIVE_BAR_T1_DELTA,
                         chassis::FIVE_BAR_T2_MIN,
                         chassis::FIVE_BAR_T2_MAX,
                         chassis::FIVE_BAR_T2_DELTA,
                         (motor1RelativePosition)*360 / M_TWOPI,
                         (motor2RelativePosition)*360 / M_TWOPI) /
                     1000;                           // check units with LUT
    currentX += fiveBarConfig.motor1toMotor2Length;  // I fucked up the table so fix it here
    currentX = -currentX;
    float currentY = tap::algorithms::interpolateLinear2D(
                         chassis::FIVE_BAR_LUT_Y,
                         chassis::FIVE_BAR_T1_MIN,
                         chassis::FIVE_BAR_T1_MAX,
                         chassis::FIVE_BAR_T1_DELTA,
                         chassis::FIVE_BAR_T2_MIN,
                         chassis::FIVE_BAR_T2_MAX,
                         chassis::FIVE_BAR_T2_DELTA,
                         (motor1RelativePosition)*360 / M_TWOPI,
                         (motor2RelativePosition)*360 / M_TWOPI) /
                     1000;
    currentPosition.setPosition(modm::Vector2f(currentX, currentY));
    // finds the angle of the joint1-tip link relative to the refernce 0 (+x ax) using trig
    float psi = motor1RelativePosition -
                atan2f(currentY, currentX + fiveBarConfig.motor1toMotor2Length / 2);
    float phi = asinf(
        sqrtf(powf(currentX + fiveBarConfig.motor1toMotor2Length / 2, 2) + powf(currentY, 2)) /
        fiveBarConfig.joint1toTipLength * sin(psi));
    currentPosition.setOrientation(motor1RelativePosition - phi);
}
}  // namespace aruwsrc::control::motion
