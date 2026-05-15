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

#include "tap/algorithms/math_user_utils.hpp"

#define ts this

using namespace tap::algorithms::transforms;

using tap::algorithms::ACCELERATION_GRAVITY;
using tap::algorithms::CMSISMat;

namespace aruwsrc::engineer::wrist
{
WristSubsystem::WristSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface& motorDifferential1,
    tap::motor::MotorInterface& motorDifferential2,
    tap::motor::MotorInterface& motorTheta3,
    tap::encoder::EncoderInterface& encoderTheta2,
    const WristConfig config)
    : tap::control::Subsystem(drivers),
      config(config),
      gravityCompConfig(std::nullopt),
      motorDifferential1(motorDifferential1),
      motorDifferential2(motorDifferential2),
      motorTheta3(motorTheta3),
      encoderTheta2(encoderTheta2),
      setpointTheta1(tap::algorithms::Angle(0)),
      setpointTheta2(0),
      setpointTheta3(
          tap::algorithms::Angle(0)),  // TODO: in theory this could be wrapped to PI/2 bc square
      pidTheta1(config.theta1PidConfig),
      pidTheta2(config.theta2PidConfig),
      pidTheta3(config.theta3PidConfig)
{
    ts->motorDifferential2 = motorDifferential2;
}

float WristSubsystem::getTheta1() const
{
    return motorDifferential1.getEncoder()->getPosition().getUnwrappedValue();
}
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

void WristSubsystem::homeTheta3(float pos) { motorTheta3.getEncoder()->resetEncoderValue(pos); }

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
    return std::abs(motorDifferential1.getEncoder()->getPosition().minDifference(setpointTheta1)) <
           epsilon;
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
    motorDifferential1.initialize();
    motorDifferential2.initialize();
    motorTheta3.initialize();
    encoderTheta2.initialize();
}

void WristSubsystem::refresh()
{
    if (!isOnline())
    {
        motorDifferential1.setDesiredOutput(0);
        motorDifferential2.setDesiredOutput(0);
        motorTheta3.setDesiredOutput(0);
        return;
    }

    float theta1Error =
        motorDifferential1.getEncoder()->getPosition().minDifference(setpointTheta1);
    float theta2Error = encoderTheta2.getPosition().minDifference(setpointTheta2);
    float errorTheta3 = motorTheta3.getEncoder()->getPosition().minDifference(setpointTheta3);

    float pidOutTheta1 = pidTheta1.runController(
        theta1Error,
        motorDifferential1.getEncoder()->getVelocity(),
        0.002f);
    float pidOutTheta2 = pidTheta2.runController(theta2Error, encoderTheta2.getVelocity(), 0.002f);
    float pidOutTheta3 =
        pidTheta3.runController(errorTheta3, motorTheta3.getEncoder()->getVelocity(), 0.002f);

    // gravity comp
    Position mountingFrameToCOM =
        gravityCompConfig->worldToMountingFrame.apply(gravityCompConfig->pointMass.location);
    Vector mountRelativeGravityForce = gravityCompConfig->worldToMountingFrame.apply(
        Vector(0, 0, -ACCELERATION_GRAVITY * gravityCompConfig->pointMass.mass));
    Vector mountRelativeGravityTorque =
        mountingFrameToCOM.toVector().cross(gravityCompConfig->worldToMountingFrame.apply(
            Vector(0, 0, -ACCELERATION_GRAVITY * gravityCompConfig->pointMass.mass)));

    float theta1 = getTheta1();
    float theta2 = getTheta2();

    float theta1GravityTorque = mountRelativeGravityTorque.x();
    float theta2GravityTorque = mountRelativeGravityTorque.y() * cosf(theta1) +
                                mountRelativeGravityTorque.z() * sinf(theta1);
    float theta3GravityTorque = mountRelativeGravityTorque.x() * cosf(theta2) +
                                mountRelativeGravityTorque.y() * sinf(theta1) * sinf(theta2) -
                                mountRelativeGravityTorque.z() * cosf(theta1) * sinf(theta2);

    // differential
    float motor1GravityTorque = theta1GravityTorque * gravityCompConfig->motor1TorqueConstant;
    float motor2GravityTorque =
        (theta1GravityTorque + theta2GravityTorque) * gravityCompConfig->motor2TorqueConstant;
    float outMotorDifferential1 = pidOutTheta1 - motor1GravityTorque;
    float outMotorDifferential2 = pidOutTheta1 + pidOutTheta2 - motor2GravityTorque;
    float outMotor3 = pidOutTheta3 - theta3GravityTorque;

    motorDifferential1.setDesiredOutput(std::clamp<int32_t>(
        outMotorDifferential1,
        -config.maxMotorDesiredOutput,
        config.maxMotorDesiredOutput));
    motorDifferential2.setDesiredOutput(std::clamp<int32_t>(
        outMotorDifferential2,
        -config.maxMotorDesiredOutput,
        config.maxMotorDesiredOutput));
    motorTheta3.setDesiredOutput(std::clamp<int32_t>(
        outMotor3,
        -config.maxMotorDesiredOutput,
        config.maxMotorDesiredOutput));
}

void WristSubsystem::refreshSafeDisconnect()
{
    motorDifferential1.setDesiredOutput(0);
    motorDifferential2.setDesiredOutput(0);
    motorTheta3.setDesiredOutput(0);
}

bool WristSubsystem::isOnline() const
{
    return motorDifferential1.isMotorOnline() && motorDifferential2.isMotorOnline() &&
           motorTheta3.isMotorOnline() && encoderTheta2.isOnline();
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