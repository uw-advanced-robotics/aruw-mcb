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

#include "balancing_chassis_subsystem.hpp"

namespace aruwsrc::chassis
{
BalancingChassisSubsystem::BalancingChassisSubsystem(
    tap::Drivers* drivers,
    BalancingLeg& leftLeg,
    BalancingLeg& rightLeg)
    : Subsystem(drivers),
      rotationPid(AUTOROTATION_PID),
      leftLeg(leftLeg),
      rightLeg(rightLeg)
{
}

void BalancingChassisSubsystem::initialize()
{
    desiredX = leftLeg.getDefaultPosition().getX();
    desiredR = 0;
    desiredZ = leftLeg.getDefaultPosition().getY();
    leftLeg.initialize();
    rightLeg.initialize();
}

void BalancingChassisSubsystem::refresh()
{
    // const uint32_t curTime = tap::arch::clock::getTimeMilliseconds();
    // const uint32_t dt = curTime - prevTime;
    // prevTime = curTime;

    // 1. Update yaw and roll values
    pitch = drivers->mpu6500.getRoll() * M_TWOPI / 360;
    roll = drivers->mpu6500.getPitch() * M_TWOPI / 360;
    computeState();

    // 2. Apply scaling and/or control laws to yaw and roll values
    float yawAdjustment = 0;
    float rollAdjustment = 0;

    // 3. Set each side's actuators to compensate appropriate for yaw and roll error
    leftLeg.setDesiredHeight(
        tap::algorithms::limitVal<float>(desiredZ + rollAdjustment, -.35, -.15));
    rightLeg.setDesiredHeight(
        tap::algorithms::limitVal<float>(desiredZ - rollAdjustment, -.35, -.15));

    yawAdjustment = desiredR * WIDTH_BETWEEN_WHEELS_Y / 2;  // m/s

    // 4. run outputs
    leftLeg.setDesiredTranslationSpeed(-yawAdjustment + desiredX);  // m/s
    rightLeg.setDesiredTranslationSpeed(yawAdjustment + desiredX);

    leftLeg.setChassisAngle(pitch);
    rightLeg.setChassisAngle(pitch);

    leftLeg.update();
    rightLeg.update();
    // do this here for safety. Only called once per subsystem
    static_cast<aruwsrc::motor::Tmotor_AK809*>(leftLeg.getFiveBar()->getMotor1())->sendCanMessage();
    static_cast<aruwsrc::motor::Tmotor_AK809*>(leftLeg.getFiveBar()->getMotor2())->sendCanMessage();
    static_cast<aruwsrc::motor::Tmotor_AK809*>(rightLeg.getFiveBar()->getMotor1())
        ->sendCanMessage();
    static_cast<aruwsrc::motor::Tmotor_AK809*>(rightLeg.getFiveBar()->getMotor2())
        ->sendCanMessage();
}

void BalancingChassisSubsystem::computeState()
{
    currentV = (rightLeg.getCurrentTranslationSpeed() + leftLeg.getCurrentTranslationSpeed()) / 2;

    float rightRot =
        2 * (rightLeg.getCurrentTranslationSpeed() - currentV) / WIDTH_BETWEEN_WHEELS_Y;
    float leftRot = 2 * (leftLeg.getCurrentTranslationSpeed() + currentV) / WIDTH_BETWEEN_WHEELS_Y;
    currentR = (rightRot + leftRot) / 2;
    currentZ = rightLeg.getCurrentHeight() > leftLeg.getCurrentHeight()
                   ? rightLeg.getCurrentHeight()
                   : leftLeg.getCurrentHeight();
}

void BalancingChassisSubsystem::runHardwareTests() {}
}  // namespace aruwsrc::chassis
