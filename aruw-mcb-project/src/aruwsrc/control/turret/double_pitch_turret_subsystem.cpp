/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "double_pitch_turret_subsystem.hpp"

#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/architecture/clock.hpp"
#include "aruwlib/drivers.hpp"
#include "aruwlib/errors/create_errors.hpp"

using namespace aruwlib::motor;
using namespace aruwlib::algorithms;
using namespace aruwlib::control::turret;
using namespace aruwlib;

namespace aruwsrc::control::turret
{
DoublePitchTurretSubsystem::DoublePitchTurretSubsystem(
    aruwlib::Drivers* drivers,
    float yawKp,
    float yawKi,
    float yawKd,
    float yawMaxICumulative,
    float yawMaxOutput,
    float yawTQDerivativeKalman,
    float yawTRDerivativeKalman,
    float yawTQProportionalKalman,
    float yawTRProportionalKalman,
    float pitchKp,
    float pitchKi,
    float pitchKd,
    float pitchMaxICumulative,
    float pitchMaxOutput,
    float pitchTQDerivativeKalman,
    float pitchTRDerivativeKalman,
    float pitchTQProportionalKalman,
    float pitchTRProportionalKalman,
    float userYawInputScalar,
    float userPitchInputScalar,
    float pitchGravityCompensationKp,
    float yawStartAngle,
    float pitchStartAngle,
    float yawMinAngle,
    float yawMaxAngle,
    float pitchMinAngle,
    float pitchMaxAngle,
    float yawStartEncoderPosition,
    float pitch90DegEncoderPositionLeft,
    float pitch90DegEncoderPositionRight,
    float feedForwardKp,
    float feedForwardMaxOutput,
    aruwlib::can::CanBus motorCanBus,
    aruwlib::motor::MotorId pitchMotorRightId,
    aruwlib::motor::MotorId pitchMotorLeftId,
    aruwlib::motor::MotorId yawMotorId,
    bool limitYaw)
    : iTurretSubsystem(drivers),
      TURRET_YAW_START_ANGLE(yawStartAngle),
      TURRET_YAW_MIN_ANGLE(yawMinAngle),
      TURRET_YAW_MAX_ANGLE(yawMaxAngle),
      TURRET_PITCH_START_ANGLE(pitchStartAngle),
      TURRET_PITCH_MIN_ANGLE(pitchMinAngle),
      TURRET_PITCH_MAX_ANGLE(pitchMaxAngle),
      USER_YAW_INPUT_SCALAR(userYawInputScalar),
      USER_PITCH_INPUT_SCALAR(userPitchInputScalar),
      PITCH_GRAVITY_COMPENSATION_KP(pitchGravityCompensationKp),
      YAW_START_ENCODER_POSITION(yawStartEncoderPosition),
      PITCH_90DEG_ENCODER_POSITION_LEFT(),
      PITCH_90DEG_ENCODER_POSITION_RIGHT(),
      currLeftPitchAngle(0.0f, 0.0f, 360.0f),
      currRightPitchAngle(0.0f, 0.0f, 360.0f),
      currYawAngle(0.0f, 0.0f, 360.0f),
      yawTarget(yawStartAngle, 0.0f, 360.0f),
      pitchTarget(pitchStartAngle, 0.0f, 360.0f),
      yawMotorPid(
          yawKp,
          yawKi,
          yawKd,
          yawMaxICumulative,
          yawMaxOutput,
          yawTQDerivativeKalman,
          yawTRDerivativeKalman,
          yawTQProportionalKalman,
          yawTRProportionalKalman),
      leftPitchPid(
          pitchKp,
          pitchKi,
          pitchKd,
          pitchMaxICumulative,
          pitchMaxOutput,
          pitchTQDerivativeKalman,
          pitchTRDerivativeKalman,
          pitchTQProportionalKalman,
          pitchTRProportionalKalman),
      rightPitchPid(
          pitchKp,
          pitchKi,
          pitchKd,
          pitchMaxICumulative,
          pitchMaxOutput,
          pitchTQDerivativeKalman,
          pitchTRDerivativeKalman,
          pitchTQProportionalKalman,
          pitchTRProportionalKalman),
      prevTime(0),
      limitYaw(limitYaw),
      pitchMotorLeft(drivers, pitchMotorRightId, motorCanBus, true, "pitch motor left"),
      pitchMotorRight(drivers, pitchMotorLeftId, motorCanBus, false, "pitch motor right"),
      yawMotor(drivers, yawMotorId, motorCanBus, true, "yaw motor")
{
}

void DoublePitchTurretSubsystem::initialize()
{
    yawMotor.initialize();
    pitchMotorLeft.initialize();
    pitchMotorRight.initialize();
}

void DoublePitchTurretSubsystem::refresh()
{
    updateCurrentTurretAngles();

    uint32_t currTime = arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    runPositionPid(currYawAngle, yawTarget, dt, 0, 0, yawMotorPid, yawMotor);
    runPositionPid(currLeftPitchAngle, pitchTarget, dt, 0, 0, leftPitchPid, pitchMotorLeft);
    runPositionPid(currRightPitchAngle, pitchTarget, dt, 0, 0, rightPitchPid, pitchMotorRight);
}

void DoublePitchTurretSubsystem::updateCurrentTurretAngles()
{
    // Update yaw angle
    updateTurretAngle(yawMotor, YAW_START_ENCODER_POSITION, TURRET_YAW_START_ANGLE, currYawAngle);

    // Update left and right pitch angle
    updateTurretAngle(pitchMotorLeft, PITCH_90DEG_ENCODER_POSITION_LEFT, 90.0f, currLeftPitchAngle);
    updateTurretAngle(
        pitchMotorRight,
        PITCH_90DEG_ENCODER_POSITION_RIGHT,
        90.0f,
        currRightPitchAngle);
}

void DoublePitchTurretSubsystem::updateTurretAngle(
    const DjiMotor& motor,
    uint16_t calibrationEncoderValue,
    float calibrationAngle,
    ContiguousFloat& turretAngle)
{
    if (motor.isMotorOnline())
    {
        turretAngle.setValue(
            DjiMotor::encoderToDegrees(
                static_cast<uint16_t>(motor.getEncoderUnwrapped() - calibrationEncoderValue)) +
            calibrationAngle);
    }
    else
    {
        turretAngle.setValue(calibrationAngle);
    }
}

void DoublePitchTurretSubsystem::runPositionPid(
    const ContiguousFloat& currAngle,
    const ContiguousFloat& setpoint,
    const uint32_t dt,
    const float errorBtwnMotors,
    const float pitchGravityCompensation,
    algorithms::TurretPid& pidController,
    DjiMotor& motor)
{
    const float positionControllerError = errorBtwnMotors + currAngle.difference(setpoint);
    const float pidOutput =
        pidController.runController(positionControllerError, getVelocity(motor), dt) +
        pitchGravityCompensation;
    setMotorOutput(pidOutput, motor);
}

float DoublePitchTurretSubsystem::getYawAngleFromCenter() const
{
    ContiguousFloat yawAngleFromCenter(
        currYawAngle.getValue() - TURRET_YAW_START_ANGLE,
        -180.0f,
        180.0f);
    return yawAngleFromCenter.getValue();
}

float DoublePitchTurretSubsystem::getPitchAngleFromCenter() const
{
    aruwlib::algorithms::ContiguousFloat yawAngleFromCenter(
        currLeftPitchAngle.getValue() - TURRET_PITCH_START_ANGLE,
        -180.0f,
        180.0f);
    return yawAngleFromCenter.getValue();
}

void DoublePitchTurretSubsystem::setMotorOutput(float out, DjiMotor& motor)
{
    out = limitVal<float>(out, INT32_MIN, INT32_MAX);

    if (motor.isMotorOnline())
    {
        motor.setDesiredOutput(out);
    }
}

const ContiguousFloat& DoublePitchTurretSubsystem::getCurrentYawValue() const
{
    return currYawAngle;
}

const ContiguousFloat& DoublePitchTurretSubsystem::getCurrentPitchValue() const
{
    /**
     * If the angles are basically always the same (which they should be) this should be fine,
     * otherwise store an averaged pitch motor angle.
     */
    return currLeftPitchAngle;
}

void DoublePitchTurretSubsystem::setYawSetpoint(float target)
{
    yawTarget.setValue(target);
    yawTarget.setValue(
        ContiguousFloat::limitValue(yawTarget, TURRET_YAW_MIN_ANGLE, TURRET_YAW_MAX_ANGLE));
}

void DoublePitchTurretSubsystem::setPitchSetpoint(float target)
{
    pitchTarget.setValue(target);
    pitchTarget.setValue(
        ContiguousFloat::limitValue(pitchTarget, TURRET_PITCH_MIN_ANGLE, TURRET_PITCH_MAX_ANGLE));
}

}  // namespace aruwsrc::control::turret
