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
    computeMotorAngleSetpoints();

    // Zero the motors. This assumes you boot at default postion

    motor1Home = motor1Setpoint;
    motor2Home = motor2Setpoint;

    motor1->initialize();
    motor2->initialize();
    motor1->setDesiredOutput(0);
    motor2->setDesiredOutput(0);
}

void FiveBarLinkage::refresh()
{
    motor1RelativePosition = motor1->getPositionUnwrapped() + motor1Home;
    motor2RelativePosition = motor2->getPositionUnwrapped() + motor2Home;

    // TODO: Replace/remove this call once motor homing is merged
    motor1->isMotorOnline();
    motor2->isMotorOnline();

    computeMotorAngleSetpoints();
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

void FiveBarLinkage::moveMotors(float motor1Output, float motor2Output)
{
    motor1->setDesiredOutput(motor1Output);
    motor2->setDesiredOutput(motor2Output);
}

void FiveBarLinkage::computeMotorAngleSetpoints()
{
    // Move the computation point from the center of the motors to motor 1

    float xp = desiredPosition.getX() +
               fiveBarConfig.motor1toMotor2Length / 2;
    
    /**
     * Define the following:
     * C1 := The vector from motor 1 to the end-effector setpoint
     * C2 := The vector from motor 2 to the end-effector setpoint
     * L  := The vector from motor 1 to motor 2 = len*x + 0*y
     * yp := y-component of C1
     * 
     * From the definitions, we know that
     * C2 = -C1 + L
     * 
     * Therefore,
     * ||C2|| = ||-C1 + L||
     * and
     * ||C2||^2 = ||-C1 + L||
     * 
     * In linkage coordinates, this turns into the following
     * 
     * ||C1||^2 = xp^2 + yp^2
     * ||C2||^2 = (-xp + len)^2 + (-yp + 0)^2
     *          = xp^2 - 2*len*xp + len^2 + yp^2
     *          = (xp^2 + yp^2) - 2*len*xp + len^2
     *          = ||C1||^2 - 2*len*xp + len^2
     * 
     * We can compute these values and then use tap::algorithms::fastInvSqrt to obtain
     * the inverse of the vector magnitudes, which will be needed in the next step for computing
     * the motor setpoints.
    */

    float magC1Squared = powf(xp, 2) + powf(desiredPosition.getY(), 2);
    float magC2Squared = magC1Squared - 2*xp*fiveBarConfig.motor1toMotor2Length + powf(fiveBarConfig.motor1toMotor2Length, 2);
    float magC1Inv = tap::algorithms::fastInvSqrt(magC1Squared), magC2Inv = tap::algorithms::fastInvSqrt(magC2Squared);

    /**
     * Define the following:
     * alpha_1 := (CCW+) angle from L to C1
     * alpha_2 := (CW+) angle from L to C2
     * beta_1  := (CCW+) angle from C1 to motor 1 link
     * beta_2  := (CW+) angle from C2 to motor 2 link
     * 
     * This results in:
     * motor1Setpoint = pi + alpha_1 + beta_1
     * motor2Setpoint = 2*pi - alpha_2 - beta_2
     * 
     * Alpha and Beta values can be computed using the motor link lengths as shown below.
    */

    float alpha_1 = acosf(xp * magC1Inv);
    float beta_1  = acosf((powf(fiveBarConfig.joint1toTipLength, 2) - powf(fiveBarConfig.motor1toJoint1Length, 2) - magC1Squared)
                        * magC1Inv / (-2 * fiveBarConfig.motor1toJoint1Length));
    float alpha_2 = acosf((-xp + fiveBarConfig.motor1toMotor2Length) * magC2Inv);
    float beta_2  = acosf((powf(fiveBarConfig.joint2toTipLength, 2) - powf(fiveBarConfig.motor2toJoint2Length, 2) - magC2Squared)
                        * magC2Inv / (-2 * fiveBarConfig.motor2toJoint2Length));

    motor1Setpoint = M_PI + alpha_1 + beta_1;
    motor2Setpoint = M_TWOPI - alpha_2 - beta_2;
}

void FiveBarLinkage::computePositionFromAngles()
{
    /** Use bilinear interpolation to compute xy-values from lookup table.
     *  See tap::algorithms for further docs.
    */
    // TODO: Replace this with bilinear interpolator when it's merged into taproot

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

    // TODO: Unfuck the table (yes Derek, I will do it for you - Manoli)
    
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
    
    // Set current position to computed values
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
