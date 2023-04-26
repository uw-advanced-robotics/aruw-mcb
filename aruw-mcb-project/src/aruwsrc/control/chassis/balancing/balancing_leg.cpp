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
    const float wheelRadius,
    const SmoothPidConfig driveWheelPidConfig)
    : drivers(drivers),
      chassisMCB(chassisMCB),
      WHEEL_RADIUS(wheelRadius),
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

    modm::Vector2f desiredWheelLocation = modm::Vector2f(0, 0.150);
    float desiredWheelAngle = 0;

    desiredWheelAngle -= fivebar->getCurrentPosition().getOrientation() - motorLinkAnglePrev;  // subtract
    motorLinkAnglePrev = fivebar->getCurrentPosition().getOrientation();

    // xoffset = xPid.runController(vDesired - vCurrent, aCurrent, dt);
    xoffset = 0;
    float desiredx = cos(-chassisMCB.getPitch()) * xoffset + sin(-chassisMCB.getPitch()) * zDesired;
    float desiredz =
        -sin(-chassisMCB.getPitch()) * xoffset + cos(-chassisMCB.getPitch()) * zDesired;

    desiredWheelLocation.setX(limitVal(desiredx, -.1f, .1f));
    desiredWheelLocation.setY(limitVal(desiredz, -.35f, -.1f));
    float tl_desired = atan2(-xoffset, -zCurrent);
    // float u = thetaLPid.runController(0 - (chassisMCB.getPitch()), tl_dot_w + chassisAngledot,
    // dt); u += thetaLdotPid.runControllerDerivateError(0 - (chassisMCB.getPitchVelocity()), dt);

    // float wheelTorque = 0.174 / cos(tl) * (14.7621 * sin(tl) - u);
    float lqrPos = -.4 * ((wheelPos - wheelPosDesired) * WHEEL_RADIUS);
    float lqrVel = -2.2785 * deadZone(chassisSpeed - vDesired, .01f);
    float lqrPitch = -39 * deadZone(chassisMCB.getPitch(), modm::toRadian(.5));
    float lqrPitchRate = -6.1564 * deadZone(chassisMCB.getPitchVelocity(), modm::toRadian(.35));

    // float lqrPitch = -39 * deadZone(tl, modm::toRadian(5));
    // float lqrPitchRate = -6.1564 * deadZone(tl_dot, modm::toRadian(0.35));
    float lqrYaw = 2.2361 * chassisYaw;
    float lqrYawRate = 1.1498 * chassisYawRate;
    float wheelTorque = -(lqrPos + lqrVel + lqrPitch + lqrPitchRate + lqrYaw + lqrYawRate);



    wheelTorque = limitVal(0.5f * wheelTorque, -2.0f, 2.0f);
    // float wheelCurrent =
    //     -(-52 * (-tl_desired + tl) - 10 * tl_dot - 5 * chassisSpeed / WHEEL_RADIUS);
    // wheelCurrent = limitVal(wheelCurrent, -5.0f, 5.0f);

    // desiredWheelSpeed -= thetaLPid.runControllerDerivateError(-tl, dt);
    // float driveWheelSpeedError = desiredWheelSpeed - WHEEL_RADIUS * realWheelSpeed;
    // float driveWheelOutput = driveWheelPid.runControllerDerivateError(driveWheelSpeedError, dt);

    /* 3. Send New Output Values to Actuators */

    // 3. Send New Output Values
    int32_t driveWheelOutput = wheelTorque / .3 * 16384 / 20;  // convert from Torque to output
    // int32_t driveWheelOutput = wheelCurrent * 16384 / 20;  // convert from i to output
    driveWheelOutput = lowPassFilter(prevOutput, driveWheelOutput, .3);
    prevOutput = driveWheelOutput;
    // debug1 = driveWheelOutput;
    driveWheel->setDesiredOutput(driveWheelOutput);
    fivebar->setDesiredPosition(desiredWheelLocation);
    fivebar->refresh();
    fivebarController(dt / 1000);
}

void BalancingLeg::fivebarController(uint32_t dt)
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

    float motor1output = fivebarMotor1Pid.runController(
        motor1error,
        fivebar->getMotor1()->getShaftRPM() * M_TWOPI / 60,
        dt);
    float motor2output = fivebarMotor2Pid.runController(
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
    tl_ddot = lowPassFilter(tl_ddot, tl_ddot_new, .05);
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
    // rad = (m/s) / (m) * (us/1e6)

    if (vDesired == 0 && vDesiredPrev != 0)
    {
        wheelPosDesired = wheelPos;
    }
    else if (vDesired != 0)
    {
        wheelPosDesired += vDesired / WHEEL_RADIUS * dt / 1'000'000;
    }
    vDesiredPrev = vDesired;

    float vCurrentTemp =
        realWheelSpeed * WHEEL_RADIUS - zCurrent * tl_dot * powf(1 / cos(tl), 2);  // m/s
    vCurrent = lowPassFilter(vCurrent, vCurrentTemp, .1);
    float aCurrentTemp = (vCurrent - vCurrentPrev) * 1'000'000 / dt;
    vCurrentPrev = vCurrent;

    aCurrentPrev = lowPassFilter(aCurrentPrev, aCurrentTemp, .02);
    aCurrent = lowPassFilter(aCurrent, aCurrentPrev, .02);

    chassisAngle = chassisMCB.getPitch();
    float chassisAngledotNew = (chassisAngle - chassisAnglePrev) * 1'000'000 / dt;
    chassisAnglePrev = chassisAngle;

    chassisAngledot = lowPassFilter(chassisAngledot, chassisAngledotNew, .5);
}
}  // namespace chassis
}  // namespace aruwsrc
