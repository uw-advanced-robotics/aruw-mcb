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
#define ts this

using namespace tap::algorithms::transforms;
using tap::algorithms::CMSISMat;

namespace aruwsrc::engineer::wrist
{
WristSubsystem::WristSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface& motorTheta2,
    tap::motor::MotorInterface& motorTheta1,
    tap::motor::MotorInterface& motorTheta3,
    tap::encoder::EncoderInterface& encoderTheta1,
    tap::encoder::EncoderInterface& encoderTheta2,
    tap::encoder::EncoderInterface& encoderTheta3,
    const WristConfig config)
    : tap::control::Subsystem(drivers),
      motorTheta2(motorTheta2),
      motorTheta1(motorTheta1),
      motorTheta3(motorTheta3),
      encoderTheta1(encoderTheta1),
      encoderTheta2(encoderTheta2),
      encoderTheta3(encoderTheta3),
      config(config),
      setpointTheta1(0),
      setpointTheta2(0),
      setpointTheta3(0),
      pidTheta1(config.theta1PidConfig),
      pidTheta2(config.theta2PidConfig),
      pidTheta3(config.theta3PidConfig)
{
    ts->motorTheta1 = motorTheta1;
}

float WristSubsystem::getTheta1() { return encoderTheta1.getPosition().getUnwrappedValue(); }
float WristSubsystem::getTheta2() { return encoderTheta2.getPosition().getUnwrappedValue(); }
float WristSubsystem::getTheta3() { return encoderTheta3.getPosition().getUnwrappedValue(); }

void WristSubsystem::setSetpointTheta1(float setpoint)
{
    setpointTheta1 = std::clamp(setpoint, config.theta1Min, config.theta1Max);
}
void WristSubsystem::setSetpointTheta2(float setpoint)
{
    setpointTheta2 = std::clamp(setpoint, config.theta2Min, config.theta2Max);
}
void WristSubsystem::setSetpointTheta3(float setpoint)
{
    setpointTheta3 = std::clamp(setpoint, config.theta3Min, config.theta3Max);
}

bool WristSubsystem::atSetpointTheta1(float epsilon)
{
    return std::abs(encoderTheta1.getPosition().minDifference(setpointTheta1)) < epsilon;
}

bool WristSubsystem::atSetpointTheta2(float epsilon)
{
    return std::abs(encoderTheta2.getPosition().minDifference(setpointTheta2)) < epsilon;
}

bool WristSubsystem::atSetpointTheta3(float epsilon)
{
    return std::abs(encoderTheta3.getPosition().minDifference(setpointTheta3)) < epsilon;
}

bool WristSubsystem::atSetpoint()
{
    return atSetpointTheta1(config.epsilon) && atSetpointTheta2(config.epsilon) &&
           atSetpointTheta3(config.epsilon);
}

float WristSubsystem::calculateTheta2MotorOutputForTheta1Theta2(
    float theta1Setpoint,
    float theta2Setpoint)
{
    float theta1Error = encoderTheta1.getPosition().minDifference(theta1Setpoint);
    float theta2Error = encoderTheta2.getPosition().minDifference(theta2Setpoint);

    float pidOutTheta1 = pidTheta1.runController(theta1Error, encoderTheta1.getVelocity(), 2.0f);
    float pidOutTheta2 = pidTheta2.runController(theta2Error, encoderTheta2.getVelocity(), 2.0f);

    return -pidOutTheta2 - pidOutTheta1;
}
float WristSubsystem::calculateTheta1MotorOutputForTheta1(float theta1Setpoint)
{
    float theta1Error = encoderTheta1.getPosition().minDifference(theta1Setpoint);
    float pidOutTheta1 = pidTheta1.runController(theta1Error, encoderTheta1.getVelocity(), 2.0f);
    return -pidOutTheta1;
}

void WristSubsystem::initialize()
{
    motorTheta2.initialize();
    motorTheta1.initialize();
    motorTheta3.initialize();
    encoderTheta1.initialize();
    encoderTheta2.initialize();
    encoderTheta3.initialize();
}

void WristSubsystem::refresh()
{
    if (!encoderTheta1.isOnline() || !encoderTheta2.isOnline() || !encoderTheta3.isOnline())
    {
        motorTheta2.setDesiredOutput(0);
        motorTheta1.setDesiredOutput(0);
        motorTheta3.setDesiredOutput(0);
        return;
    }

    float outMotorTheta2 =
        calculateTheta2MotorOutputForTheta1Theta2(setpointTheta1, setpointTheta2);
    float outMotorTheta1 = calculateTheta1MotorOutputForTheta1(setpointTheta1);

    float errorTheta3 = encoderTheta3.getPosition().minDifference(setpointTheta3);
    float outMotorTheta3 = pidTheta3.runController(errorTheta3, encoderTheta3.getVelocity(), 2.0f);

    motorTheta2.setDesiredOutput(std::clamp<int32_t>(
        outMotorTheta2,
        -config.maxMotorDesiredOutput,
        config.maxMotorDesiredOutput));
    motorTheta1.setDesiredOutput(std::clamp<int32_t>(
        outMotorTheta1,
        -config.maxMotorDesiredOutput,
        config.maxMotorDesiredOutput));
    motorTheta3.setDesiredOutput(std::clamp<int32_t>(
        outMotorTheta3,
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