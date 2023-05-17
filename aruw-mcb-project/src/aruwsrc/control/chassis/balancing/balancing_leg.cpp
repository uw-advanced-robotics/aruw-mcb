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

using namespace tap::algorithms;

namespace aruwsrc
{
namespace chassis
{
BalancingLeg::BalancingLeg(
    tap::Drivers* drivers,
    aruwsrc::can::TurretMCBCanComm& chassisMCB,
    aruwsrc::control::motion::FiveBarLinkage* fivebar,
    const SmoothPidConfig fivebarMotor1PidConfig,
    const FuzzyPDConfig fivebarMotor1FuzzyPDconfig,
    const SmoothPidConfig fivebarMotor2PidConfig,
    const FuzzyPDConfig fivebarMotor2FuzzyPDconfig,
    tap::motor::MotorInterface* wheelMotor,
    const SmoothPidConfig driveWheelPidConfig)
    : drivers(drivers),
      chassisMCB(chassisMCB),
      fivebar(fivebar),
      driveWheel(wheelMotor),
      fivebarMotor1Pid(fivebarMotor1FuzzyPDconfig, fivebarMotor1PidConfig),
      fivebarMotor2Pid(fivebarMotor2FuzzyPDconfig, fivebarMotor2PidConfig),
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
    /* 1. Compute dt and Update Current State */

    uint32_t currentTime = tap::arch::clock::getTimeMicroseconds();
    int32_t dt = currentTime - prevTime;
    prevTime = currentTime;
    computeState(dt);

    /* 2. Compute Setpoints */

    modm::Vector2f desiredWheelLocation = modm::Vector2f(0, 0.115);
    float desiredWheelAngle = 0;

    desiredWheelAngle -=
        fivebar->getCurrentPosition().getOrientation() - motorLinkAnglePrev;  // subtract
    motorLinkAnglePrev = fivebar->getCurrentPosition().getOrientation();

    xoffset = xPid.runControllerDerivateError(vDesired - vCurrent, dt);
    // xoffset = .00;
    float tl_desired = atan2(-xoffset, -zCurrent);
    tl_desired = 0;

    float desiredx = cos(-chassisAngle) * xoffset + sin(-chassisAngle) * zDesired;
    float desiredz = -sin(-chassisAngle) * xoffset + cos(-chassisAngle) * zDesired;
    if (!isFallen)
    {
        desiredWheelLocation.setX(limitVal(desiredx, -.1f, .1f));
        desiredWheelLocation.setY(limitVal(desiredz, -.35f, -.1f));
    }
    else
    {
        desiredWheelLocation.setX(fivebar->getDefaultPosition().getX());
        desiredWheelLocation.setY(fivebar->getDefaultPosition().getY());
    }

    float LQR_K2 = HEIGHT_TO_LQR_K2_INTERPOLATOR.interpolate(-zCurrent);
    float LQR_K3 = HEIGHT_TO_LQR_K3_INTERPOLATOR.interpolate(-zCurrent);
    float LQR_K4 = HEIGHT_TO_LQR_K4_INTERPOLATOR.interpolate(-zCurrent);

    float lqrPos = LQR_K1 * ((chassisPos - chassisPosDesired) * WHEEL_RADIUS);
    float lqrVel = LQR_K2 * deadZone(chassisSpeed - vDesired, .0f);
    float lqrPitch = deadZone(LQR_K3 * (chassisAngle), .04f);
    float lqrPitchRate = deadZone(LQR_K4 * chassisAngledot, 1.5f);
    // float lqrPitch = -38 * deadZone(tl - tl_desired, modm::toRadian(.05));
    // float lqrPitchRate = -6.3527 * deadZone(tl_dot, .2f);
    float lqrYaw = LQR_K5 * chassisYaw;
    float lqrYawRate = LQR_K6 * chassisYawRate;
    float wheelTorque = -(lqrPos + lqrVel + lqrPitch + lqrPitchRate + lqrYaw + lqrYawRate);

    debug1 = lqrPos;
    debug2 = lqrVel;
    debug3 = lqrPitch;
    debug4 = lqrPitchRate;
    debug5 = lqrYaw;
    debug6 = lqrYawRate;

    wheelTorque = limitVal(wheelTorque, -3.0f, 3.0f);
    // 3. Send New Output Values
    int32_t driveWheelOutput = wheelTorque / .3 * 16384 / 20;  // convert from Torque to output
    // int32_t driveWheelOutput = wheelCurrent * 16384 / 20;  // convert from i to output
    driveWheelOutput = lowPassFilter(prevOutput, driveWheelOutput, 1);
    prevOutput = driveWheelOutput;
    debug7 = driveWheelOutput;
    if (armed && !isFallen)
    {
        driveWheel->setDesiredOutput(driveWheelOutput);
    }
    else
    {
        driveWheel->setDesiredOutput(0);
        desiredWheelLocation = fivebar->getDefaultPosition();
    }
    zDesRamper.setTarget(desiredWheelLocation.getY());
    zDesRamper.update(Z_RAMP_RATE * dt / 1'000'000);
    desiredWheelLocation.setY(limitVal(zDesRamper.getValue(), -.35f, -.1f));

    fivebar->setDesiredPosition(desiredWheelLocation);
    fivebar->refresh();
    fivebarController(dt / 1000);
}

void BalancingLeg::fivebarController(uint32_t dt)
{
    float L = fivebar->getFiveBarConfig().motor1toMotor2Length;
    float B = chassisAngle;
    float gravT1 = fivebar->getFiveBarConfig().motor1toJoint1Length *
                   cos(fivebar->getMotor1RelativePosition()) *
                   ((fivebar->getCurrentPosition().getX() + (L * cos(B) / 2)) / L * cos(B)) *
                   (MASS_CHASSIS * 9.81 / 2);
    float gravT2 = fivebar->getFiveBarConfig().motor2toJoint2Length *
                   cos(M_PI - fivebar->getMotor2RelativePosition()) *
                   (-(fivebar->getCurrentPosition().getX() - (L * cos(B) / 2)) / L * cos(B)) *
                   (MASS_CHASSIS * 9.81 / 2);

    float motor1error = fivebar->getMotor1Error();
    float motor2error = fivebar->getMotor2Error();

    float motor1output = fivebarMotor1Pid.runController(
        motor1error,
        fivebar->getMotor1()->getShaftRPM() * M_TWOPI / 60,
        dt);
    float motor2output = fivebarMotor2Pid.runController(
        motor2error,
        fivebar->getMotor2()->getShaftRPM() * M_TWOPI / 60,
        dt);
    if (!isFallen && armed)
    {
        motor1output -= 1000 * gravT1 / aruwsrc::motor::AK809_TORQUE_CONSTANT;
        // motor direction so minus
        motor2output += 1000 * gravT2 / aruwsrc::motor::AK809_TORQUE_CONSTANT;
    }
    fivebar->moveMotors(motor1output, motor2output);
}

void BalancingLeg::computeState(uint32_t dt)
{
    zCurrent = fivebar->getCurrentPosition().getX() * sin(chassisAngle) +
               fivebar->getCurrentPosition().getY() * cos(chassisAngle);
    float x_l = fivebar->getCurrentPosition().getX() * cos(chassisAngle) +
                fivebar->getCurrentPosition().getY() * sin(chassisAngle);
    tl = atan2(-x_l, -zCurrent - WHEEL_RADIUS / 2);  // rad

    // dt is in us, 1000 to get s
    float tl_dot1 = 1'000'000.0f / static_cast<float>(dt) * (tl - tl_prev);            // rad/s
    float tl_dot2 = 1'000'000.0f / static_cast<float>(dt) * (tl_prev - tl_prev_prev);  // rad/s
    float tl_dot3 = 1'000'000.0f / static_cast<float>(2 * dt) * (tl - tl_prev_prev);   // rad/s
    tl_dot = lowPassFilter(tl_dot, (tl_dot1 + tl_dot2 + tl_dot3) / 3, .1);             // Avg of 3

    // tl_dotPrev = lowPassFilter(tl_dotPrev, tl_dot, .1);
    // tl_dotPrev = lowPassFilter(tl_dotPrev, tl_dot, .1);
    // tl_dot = lowPassFilter(tl_dotPrev, tl_dot, .1);
    float tl_ddot_new = (tl - 2 * tl_prev + tl_prev_prev) /
                        powf((static_cast<float>(dt) / 1'000'000.0f), 2);  // rad/s/s
    tl_ddot = lowPassFilter(tl_ddot, tl_ddot_new, .1);
    // tl_ddot = tl_ddot_new;

    tl_prev_prev = tl_prev;
    tl_prev = tl;
    tl_dotPrev = tl_dot;
    tl_ddotPrev = tl_ddot;

    wheelPos = driveWheel->getPositionUnwrapped() * CHASSIS_GEARBOX_RATIO;  // rad
    realWheelSpeedPrev = lowPassFilter(
        realWheelSpeed,
        1'000'000 * (wheelPos - wheelPosPrev) / dt,
        .1);  // rad/s
    realWheelSpeed = lowPassFilter(realWheelSpeed, realWheelSpeedPrev, .3);
    wheelPosPrev = wheelPos;

    float vCurrentTemp =
        realWheelSpeed * WHEEL_RADIUS;  // m/s  - zCurrent * tl_dot * powf(1 / cos(tl), 2)
    vCurrent = lowPassFilter(vCurrent, vCurrentTemp, .1);
    float aCurrentTemp = (vCurrent - vCurrentPrev) * 1'000'000 / dt;
    vCurrentPrev = vCurrent;

    aCurrentPrev = lowPassFilter(aCurrentPrev, aCurrentTemp, .02);
    aCurrent = lowPassFilter(aCurrent, aCurrentPrev, .02);
    iveFallenAndICantGetUp();
}

void BalancingLeg::iveFallenAndICantGetUp()
{
    if (abs(chassisAngle) > abs(FALLEN_ANGLE_THRESHOLD))
    {
        isFallen = true;
    }
    else if (
        abs(chassisAngle) < abs(FALLEN_ANGLE_RETURN) &&
        abs(chassisAngledot) < abs(FALLEN_ANGLE_RATE_THRESHOLD))
    {
        isFallen = false;
    }
}

}  // namespace chassis
}  // namespace aruwsrc
