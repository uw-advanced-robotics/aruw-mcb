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
    const WristConfig config)
    : tap::control::Subsystem(drivers),
      motorLeft(motorLeft),
      motorRight(motorRight),
      encoderPitch(encoderPitch),
      encoderYaw(encoderYaw),
      pidPitch(config.pitchPidConfig),
      pidYaw(config.yawPidConfig),
      config(config),
      setpointPitch(0),
      setpointYaw(0),
      leadControllerPitch(
          tap::algorithms::filter::phaseLagLeadCoefficients(10000, 5, 25, 1 / 500.0)),
      lagControllerPitch(
          tap::algorithms::filter::phaseLagLeadCoefficients(1.75, 24, 25, 1 / 500.0)),
      leadControllerYaw(tap::algorithms::filter::phaseLagLeadCoefficients(10000, 5, 25, 1 / 500.0)),
      lagControllerYaw(tap::algorithms::filter::phaseLagLeadCoefficients(1.75, 24, 25, 1 / 500.0))
{
}

void WristSubsystem::setSetpointPitch(float setpoint)
{
    if (config.minPitch == config.maxPitch)
        setpointPitch = setpoint;
    else
        setpointPitch = std::clamp(setpoint, config.minPitch, config.maxPitch);
}

void WristSubsystem::setSetpointYaw(float setpoint)
{
    if (config.minYaw == config.maxYaw)
        setpointYaw = setpoint;
    else
        setpointYaw = std::clamp(setpoint, config.minYaw, config.maxYaw);
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

bool WristSubsystem::atSetpoint()
{
    return atSetpointPitch(config.epsilon) && atSetpointYaw(config.epsilon);
}

void WristSubsystem::initialize()
{
    motorLeft.initialize();
    motorRight.initialize();
    encoderPitch.initialize();
    encoderYaw.initialize();
}

void WristSubsystem::refresh()
{
    if (!encoderPitch.isOnline() || !encoderYaw.isOnline())
    {
        motorLeft.setDesiredOutput(0);
        motorRight.setDesiredOutput(0);
        return;
    }

    CMSISMat<3, 1> gantryToCOMTranslation =
        computeWristToCOM(getYaw(), getPitch(), COM_POS).getTranslation().coordinates();

    Vector gravityTorque(tap::algorithms::cross(
        gantryToCOMTranslation,
        CMSISMat<3, 1>({0, 0, -9.8f * WRIST_MASS_KG})));

    // we can compute the torque exerted on each joint by projecting the robot-space gravity torque
    // into the joint axis subspace
    Vector pitchAxis(0, 1, 0);
    Vector yawAxis = Transform(0, 0, 0, 0, -getPitch(), 0).apply(Vector(0, 0, 1));

    // torque applied on each joint by gravity
    float gravityPitchTorque = gravityTorque.dot(pitchAxis);
    float gravityYawTorque = gravityTorque.dot(yawAxis);

    // gravity torque halved because we have two motors
    float lead =
        leadControllerPitch.filterData(encoderPitch.getPosition().minDifference(setpointPitch));

    float outPitch =
        lagControllerPitch.filterData(lead) - gravityPitchTorque / 2 * M3508_TORQUE_CONSTANT;

    // gear ratio only applied to gravity compensation here because pid was tuned without it
    // gravity torque halved because we have two motors

    float yawLead =
        leadControllerYaw.filterData(encoderYaw.getPosition().minDifference(setpointYaw));

    float outYaw = lagControllerYaw.filterData(yawLead) -
                   gravityYawTorque / 2 * M3508_TORQUE_CONSTANT * config.ratio;
    // differential
    float outLeft = outYaw + outPitch;
    float outRight = outYaw - outPitch;

    motorLeft.setDesiredOutput(
        std::clamp<int32_t>(outLeft, -config.maxMotorDesiredOutput, config.maxMotorDesiredOutput));
    motorRight.setDesiredOutput(
        std::clamp<int32_t>(outRight, -config.maxMotorDesiredOutput, config.maxMotorDesiredOutput));
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