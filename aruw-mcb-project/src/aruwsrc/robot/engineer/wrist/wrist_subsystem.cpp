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

#include "wrist_subsystem.hpp"
#define ts this

using namespace tap::algorithms::transforms;
using tap::algorithms::CMSISMat;

namespace aruwsrc::engineer::wrist
{
WristSubsystem::WristSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface& motorTheta1,
    tap::motor::MotorInterface& motorTheta2,
    tap::motor::MotorInterface& motorTheta3,
    tap::encoder::EncoderInterface& encoderTheta1,
    tap::encoder::EncoderInterface& encoderTheta2,
    const WristConfig config)
    : config(config),
      tap::control::Subsystem(drivers),
      motorTheta1(motorTheta1),
      motorTheta2(motorTheta2),
      motorTheta3(motorTheta3),
      encoderTheta1(encoderTheta1),
      encoderTheta2(encoderTheta2),
      setpointTheta1(tap::algorithms::Angle(0)),
      setpointTheta2(0),
      setpointTheta3(
          tap::algorithms::Angle(0)),  // TODO: in theory this could be wrapped to PI/2 bc square
      pidTheta1(config.theta1PidConfig),
      pidTheta2(config.theta2PidConfig),
      pidTheta3(config.theta3PidConfig)
{
    ts->motorTheta1 = motorTheta1;
}

float WristSubsystem::getTheta1() const { return encoderTheta1.getPosition().getUnwrappedValue(); }
float WristSubsystem::getTheta2() const { return encoderTheta2.getPosition().getUnwrappedValue(); }
float WristSubsystem::getTheta3() const
{
    return motorTheta3.getEncoder()->getPosition().getUnwrappedValue();
}

void WristSubsystem::setSetpointTheta1(float setpoint)
{
    setpointTheta1.setUnwrappedValue(setpoint);
}
void WristSubsystem::setSetpointTheta2(float setpoint)
{
    setpointTheta2 = std::clamp(setpoint, config.theta2Min, config.theta2Max);
}
void WristSubsystem::setSetpointTheta3(float setpoint)
{
    setpointTheta3.setUnwrappedValue(setpoint);
}

void WristSubsystem::setSetpointOrientation(tap::algorithms::transforms::Orientation setpoint)
{
    float theta1 = atan2f(setpoint.matrix().data[3], -setpoint.matrix().data[6]);
    float theta2 = acosf(setpoint.matrix().data[0]);
    float theta3 = atan2f(setpoint.matrix().data[1], -setpoint.matrix().data[2]);
    setSetpointTheta1(theta1);
    setSetpointTheta2(theta2);
    setSetpointTheta3(theta3);
    // TODO: handle gimbal lock
}

bool WristSubsystem::atSetpointTheta1(float epsilon) const
{
    return std::abs(encoderTheta1.getPosition().minDifference(setpointTheta1)) < epsilon;
}

bool WristSubsystem::atSetpointTheta2(float epsilon) const
{
    return std::abs(encoderTheta2.getPosition().minDifference(setpointTheta2)) < epsilon;
}

bool WristSubsystem::atSetpointTheta3(float epsilon) const
{
    return std::abs(motorTheta3.getEncoder()->getPosition().minDifference(setpointTheta3)) <
           epsilon;
}

bool WristSubsystem::atSetpoint() const
{
    return atSetpointTheta1(config.epsilon) && atSetpointTheta2(config.epsilon) &&
           atSetpointTheta3(config.epsilon);
}

void WristSubsystem::initialize()
{
    motorTheta2.initialize();
    motorTheta1.initialize();
    motorTheta3.initialize();
    encoderTheta1.initialize();
    encoderTheta2.initialize();
}

void WristSubsystem::refresh()
{
    if (!isOnline())
    {
        motorTheta2.setDesiredOutput(0);
        motorTheta1.setDesiredOutput(0);
        motorTheta3.setDesiredOutput(0);
        return;
    }

    float theta1Error = encoderTheta1.getPosition().minDifference(setpointTheta1);
    float theta2Error = encoderTheta2.getPosition().minDifference(setpointTheta2);
    float errorTheta3 = motorTheta3.getEncoder()->getPosition().minDifference(setpointTheta3);

    float pidOutTheta1 = pidTheta1.runController(theta1Error, encoderTheta1.getVelocity(), 2.0f);
    float pidOutTheta2 = pidTheta2.runController(theta2Error, encoderTheta2.getVelocity(), 2.0f);
    float pidOutTheta3 =
        pidTheta3.runController(errorTheta3, motorTheta3.getEncoder()->getVelocity(), 2.0f);

    float outMotorTheta2 = -pidOutTheta2 - pidOutTheta1;
    float outMotorTheta1 = -pidOutTheta1;

    motorTheta2.setDesiredOutput(std::clamp<int32_t>(
        outMotorTheta2,
        -config.maxMotorDesiredOutput,
        config.maxMotorDesiredOutput));
    motorTheta1.setDesiredOutput(std::clamp<int32_t>(
        outMotorTheta1,
        -config.maxMotorDesiredOutput,
        config.maxMotorDesiredOutput));
    motorTheta3.setDesiredOutput(std::clamp<int32_t>(
        pidOutTheta3,
        -config.maxMotorDesiredOutput,
        config.maxMotorDesiredOutput));

    // CMSISMat<3, 1> gantryToCOMTranslation =
    //     computeWristToCOM(getYaw(), getPitch(), COM_POS).getTranslation().coordinates();

    // Vector gravityTorque(
    //     tap::algorithms::cross(
    //         gantryToCOMTranslation,
    //         CMSISMat<3, 1>({0, 0, -9.8f * WRIST_MASS_KG})));

    // // we can compute the torque exerted on each joint by projecting the robot-space gravity
    // torque
    // // into the joint axis subspace
    // Vector pitchAxis(0, 1, 0);
    // Vector yawAxis = Transform(0, 0, 0, 0, -getPitch(), 0).apply(Vector(0, 0, 1));

    // // torque applied on each joint by gravity
    // float gravityPitchTorque = gravityTorque.dot(pitchAxis);
    // float gravityYawTorque = gravityTorque.dot(yawAxis);

    // // gravity torque halved because we have two motors
    // float outPitch = pidPitch.runController(
    //                      encoderPitch.getPosition().minDifference(setpointPitch),
    //                      encoderPitch.getVelocity(),
    //                      2.0f) -
    //                  gravityPitchTorque / 2 * M3508_TORQUE_CONSTANT;

    // // gear ratio only applied to gravity compensation here because pid was tuned without it
    // // gravity torque halved because we have two motors
    // float outYaw = pidYaw.runController(
    //                    encoderYaw.getPosition().minDifference(setpointYaw),
    //                    encoderYaw.getVelocity(),
    //                    2.0f) -
    //                gravityYawTorque / 2 * M3508_TORQUE_CONSTANT * config.ratio;

    // // differential
    // float outLeft = outYaw + outPitch;
    // float outRight = outYaw - outPitch;

    // motorLeft.setDesiredOutput(
    //     std::clamp<int32_t>(outLeft, -config.maxMotorDesiredOutput,
    //     config.maxMotorDesiredOutput));
    // motorRight.setDesiredOutput(
    //     std::clamp<int32_t>(outRight, -config.maxMotorDesiredOutput,
    //     config.maxMotorDesiredOutput));
}

void WristSubsystem::refreshSafeDisconnect()
{
    motorTheta1.setDesiredOutput(0);
    motorTheta2.setDesiredOutput(0);
    motorTheta3.setDesiredOutput(0);
}

bool WristSubsystem::isOnline() const
{
    return motorTheta2.isMotorOnline() && motorTheta1.isMotorOnline() &&
           motorTheta3.isMotorOnline() && encoderTheta1.isOnline() && encoderTheta2.isOnline();
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

Orientation WristSubsystem::getOrientation() const
{
    return getHypotheticalOrientation(getTheta1(), getTheta2(), getTheta3());
}

Orientation WristSubsystem::getHypotheticalOrientation(float theta1, float theta2, float theta3)
{
    float s1 = sinf(theta1), c1 = cosf(theta1);
    float s2 = sinf(theta2), c2 = cosf(theta2);
    float s3 = sinf(theta3), c3 = cosf(theta3);

    float c1c3 = c1 * c3;
    float s1c3 = s1 * c3;
    float c1c2 = c1 * c2;
    float s1c2 = s1 * c2;

    return Orientation(tap::algorithms::CMSISMat<3, 3>(
        {c2,
         s2 * s3,
         s2 * c3,

         s1 * s2,
         -s1 * s3 * c2 + c1c3,
         -s1c2 * c3 - s3 * c1,

         -c1 * s2,
         s1c3 + s3 * c1c2,
         -s1 * s3 + c1c2 * c3}));
}
}  // namespace aruwsrc::engineer::wrist