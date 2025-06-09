/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "aruwsrc/robot/engineer/wrist/wrist_subsystem.hpp"

using namespace tap::algorithms::transforms;
using tap::algorithms::CMSISMat;

namespace aruwsrc::engineer::wrist
{
WristSubsystem::WristSubsystem(
    tap::Drivers *drivers,
    tap::motor::MotorInterface &motorLeft,
    tap::motor::MotorInterface &motorRight,
    tap::encoder::EncoderInterface &encoderPitch,
    tap::encoder::EncoderInterface &encoderYaw,
    const tap::algorithms::SmoothPidConfig configPitch,
    const tap::algorithms::SmoothPidConfig configYaw,
    float minPitch,
    float maxPitch,
    float minYaw,
    float maxYaw,
    float ratio,
    float kS,
    float epsilon)
    : tap::control::Subsystem(drivers),
      motorLeft(motorLeft),
      motorRight(motorRight),
      encoderPitch(encoderPitch),
      encoderYaw(encoderYaw),
      pidPitch(configPitch),
      pidYaw(configYaw),
      minPitch(minPitch),
      maxPitch(maxPitch),
      minYaw(minYaw),
      maxYaw(maxYaw),
      ratio(ratio),
      kS(kS),
      epsilon(epsilon)
{
    setpointPitch = 0;
    setpointYaw = 0;
}

void WristSubsystem::setSetpointPitch(float setpoint)
{
    if (minPitch == maxPitch)
        setpointPitch = setpoint;
    else
        setpointPitch = std::clamp(setpoint, minPitch, maxPitch);
}

void WristSubsystem::setSetpointYaw(float setpoint)
{
    if (minYaw == maxYaw)
        setpointYaw = setpoint;
    else
        setpointYaw = std::clamp(setpoint, minYaw, maxYaw);
}

float WristSubsystem::getPitch() { return encoderPitch.getPosition().getUnwrappedValue(); }

float WristSubsystem::getYaw() { return encoderYaw.getPosition().getUnwrappedValue(); }

bool WristSubsystem::atSetpointPitch(float epsilon)
{
    return tap::algorithms::compareFloatClose(setpointPitch, getPitch(), epsilon);
}

bool WristSubsystem::atSetpointYaw(float epsilon)
{
    return tap::algorithms::compareFloatClose(setpointYaw, getYaw(), epsilon);
}

bool WristSubsystem::atSetpoint() { return atSetpointPitch(epsilon) && atSetpointYaw(epsilon); }

void WristSubsystem::initialize()
{
    motorLeft.initialize();
    motorRight.initialize();
    encoderPitch.initialize();
    encoderYaw.initialize();
}

CMSISMat<3, 1> gantryToCOMTranslation({0, 0, 0});
float gravityPitchTorque, gravityYawTorque;
Vector yawAxis(0, 0, 0);
void WristSubsystem::refresh()
{
    if (!encoderPitch.isOnline() || !encoderYaw.isOnline())
    {
        motorLeft.setDesiredOutput(0);
        motorRight.setDesiredOutput(0);
        return;
    }

    // gravity compensation
    gantryToCOMTranslation =
        computeWristToCOM(getYaw(), getPitch(), COM_POS).getTranslation().coordinates();

    Vector gravityTorque(
        tap::algorithms::cross(
            gantryToCOMTranslation,
            CMSISMat<3, 1>({0, 0, -9.8f * WRIST_MASS_KG})));

    // we can compute the torque exerted on each joint by projecting the robot-space gravity torque
    // into the joint axis subspace
    Vector pitchAxis(0, 1, 0);
    yawAxis = Transform(0, 0, 0, 0, getPitch(), 0).apply(Vector(1, 0, 0));

    gravityPitchTorque = gravityTorque.dot(pitchAxis);
    gravityYawTorque = gravityTorque.dot(yawAxis);

    // pid

    // gravity torque halved because we have two motors
    float outPitch = pidPitch.runController(
                         encoderPitch.getPosition().minDifference(setpointPitch),
                         encoderPitch.getVelocity(),
                         2.0f) -
                     gravityPitchTorque / 2 * M3508_TORQUE_CONSTANT * PITCH_GRAVITY_SCALAR;

    // gear ratio only applied to gravity compensation here because pid was tuned without it
    // gravity torque halved because we have two motors
    float outYaw = pidYaw.runController(setpointYaw - getYaw(), encoderYaw.getVelocity(), 2.0f) -
                   gravityYawTorque / 2 * M3508_TORQUE_CONSTANT / ratio * YAW_GRAVITY_SCALAR;

    // differential
    float outLeft = outYaw + outPitch;
    float outRight = outYaw - outPitch;

    motorLeft.setDesiredOutput(outLeft + kS);
    motorRight.setDesiredOutput(outRight + kS);
}

void WristSubsystem::refreshSafeDisconnect()
{
    motorLeft.setDesiredOutput(0);
    motorRight.setDesiredOutput(0);
}

Transform WristSubsystem::computeWristToCOM(
    float yawJoint,
    float pitchJoint,
    tap::algorithms::transforms::Position COMPos) const
{
    Transform wristOrientation(
        tap::algorithms::CMSISMat<3, 1>({0, 0, 0}),
        tap::algorithms::CMSISMat<3, 3>(
            {cosf(pitchJoint) * cosf(yawJoint),
             -cosf(yawJoint) * sinf(pitchJoint),
             sinf(pitchJoint),
             sinf(yawJoint),
             cosf(yawJoint),
             0,
             -sinf(pitchJoint) * cosf(yawJoint),
             -sinf(pitchJoint) * sinf(yawJoint),
             cosf(pitchJoint)}));

    return wristOrientation.compose(Transform(COMPos, Orientation(0, 0, 0)));
}
}  // namespace aruwsrc::engineer::wrist