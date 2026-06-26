/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "drone_turret_vector_command.hpp"

#include <cmath>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"

namespace aruwsrc::drone
{
using tap::communication::sensors::imu::ImuInterface;

DroneTurretVectorCommand::DroneTurretVectorCommand(
    aruwsrc::control::ControlOperatorInterface& controlOperatorInterface,
    DroneTurretSubsystem& turret,
    const DroneIMU& turretImu,
    tap::algorithms::SmoothPid& yawPositionPid,
    tap::algorithms::SmoothPid& yawVelocityPid,
    tap::algorithms::SmoothPid& pitchPositionPid,
    tap::algorithms::SmoothPid& pitchVelocityPid,
    aruwsrc::control::turret::algorithms::ChassisFrameTurretController<
        tap::algorithms::transforms::Axis::YAW>& chassisFrameYawController,
    aruwsrc::control::turret::algorithms::ChassisFrameTurretController<
        tap::algorithms::transforms::Axis::PITCH>& chassisFramePitchController,
    float userYawInputScalar,
    float userPitchInputScalar,
    uint8_t turretID)
    : controlOperatorInterface(controlOperatorInterface),
      turret(turret),
      turretImu(turretImu),
      yawPositionPid(yawPositionPid),
      yawVelocityPid(yawVelocityPid),
      pitchPositionPid(pitchPositionPid),
      pitchVelocityPid(pitchVelocityPid),
      chassisFrameYawController(chassisFrameYawController),
      chassisFramePitchController(chassisFramePitchController),
      userYawInputScalar(userYawInputScalar),
      userPitchInputScalar(userPitchInputScalar),
      turretID(turretID)
{
    addSubsystemRequirement(&turret);
}

bool DroneTurretVectorCommand::isReady() { return !isFinished(); }

void DroneTurretVectorCommand::initialize()
{
    yawPositionPid.reset();
    yawVelocityPid.reset();
    pitchPositionPid.reset();
    pitchVelocityPid.reset();
    chassisFrameYawController.initialize();
    chassisFramePitchController.initialize();
    desiredForwardWorldFrame = getTurretForwardVectorWorldFrame();
    lastPitchInputAxisWorldFrame = getTurretPitchAxisWorldFrame();
    lastYawAxisWorldFrame =
        getTurretYawAxisWorldFrame(desiredForwardWorldFrame, lastPitchInputAxisWorldFrame);
    usingChassisFrameFallback = false;
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

void DroneTurretVectorCommand::execute()
{
    const uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    const float dt = static_cast<float>(currTime - prevTime) / 1000.0f;
    prevTime = currTime;

    const float yawInput =
        userYawInputScalar * controlOperatorInterface.getTurretYawInput(turretID);
    const float pitchInput =
        userPitchInputScalar * controlOperatorInterface.getTurretPitchInput(turretID);

    if (!turretImu.isOnline() || turretImu.getImuState() != ImuInterface::ImuState::IMU_CALIBRATED)
    {
        runChassisFrameFallback(yawInput, pitchInput, dt);
        return;
    }

    if (usingChassisFrameFallback)
    {
        yawPositionPid.reset();
        yawVelocityPid.reset();
        pitchPositionPid.reset();
        pitchVelocityPid.reset();
        desiredForwardWorldFrame = getTurretForwardVectorWorldFrame();
        usingChassisFrameFallback = false;
    }

    runWorldFrameControl(yawInput, pitchInput, dt);
}

void DroneTurretVectorCommand::runWorldFrameControl(float yawInput, float pitchInput, float dt)
{
    const Vector currentForwardWorldFrame = getTurretForwardVectorWorldFrame();
    lastPitchInputAxisWorldFrame = getTurretPitchAxisWorldFrame();
    lastYawAxisWorldFrame =
        getTurretYawAxisWorldFrame(currentForwardWorldFrame, lastPitchInputAxisWorldFrame);
    const Vector yawControlAxis =
        lastYawAxisWorldFrame -
        currentForwardWorldFrame * lastYawAxisWorldFrame.dot(currentForwardWorldFrame);
    const Vector pitchControlAxis =
        lastPitchInputAxisWorldFrame -
        currentForwardWorldFrame * lastPitchInputAxisWorldFrame.dot(currentForwardWorldFrame);
    const float yawAxisMagnitudeSquared = yawControlAxis.dot(yawControlAxis);
    const float pitchAxisMagnitudeSquared = pitchControlAxis.dot(pitchControlAxis);
    const bool yawHasPointingAuthority = yawAxisMagnitudeSquared >= MIN_YAW_AXIS_AUTHORITY;
    const float limitedYawInput = limitUserInputAtMotorLimits(yawInput, turret.yawMotor);
    const float limitedPitchInput = limitUserInputAtMotorLimits(pitchInput, turret.pitchMotor);

    if (yawHasPointingAuthority)
    {
        desiredForwardWorldFrame =
            rotateVector(desiredForwardWorldFrame, lastYawAxisWorldFrame, limitedYawInput);
    }
    desiredForwardWorldFrame =
        rotateVector(desiredForwardWorldFrame, lastPitchInputAxisWorldFrame, limitedPitchInput);
    desiredForwardWorldFrame =
        normalizeOrFallback(desiredForwardWorldFrame, currentForwardWorldFrame);

    const Vector rotationError = currentForwardWorldFrame.cross(desiredForwardWorldFrame);
    const float yawAxisError = yawControlAxis.dot(rotationError);
    const float pitchAxisError = pitchControlAxis.dot(rotationError);
    // Coupled two-axis solve. This is mathematically neat, but it made the drone turret limit-cycle
    // in practice, so the active controller below uses decoupled per-axis projection for now.
    // const float axisCoupling = yawControlAxis.dot(pitchControlAxis);
    // const float solveDeterminant = (yawAxisMagnitudeSquared + AXIS_SOLVE_DAMPING) *
    //                                    (pitchAxisMagnitudeSquared + AXIS_SOLVE_DAMPING) -
    //                                axisCoupling * axisCoupling;

    const float yawError =
        yawHasPointingAuthority
            ? clampControllerError(
                  yawAxisError / (yawAxisMagnitudeSquared + AXIS_SOLVE_DAMPING),
                  // ((pitchAxisMagnitudeSquared + AXIS_SOLVE_DAMPING) * yawAxisError -
                  //  axisCoupling * pitchAxisError) /
                  //     solveDeterminant,
                  turret.yawMotor)
            : 0.0f;
    const float pitchError = clampControllerError(
        pitchAxisError / (pitchAxisMagnitudeSquared + AXIS_SOLVE_DAMPING),
        // ((yawAxisMagnitudeSquared + AXIS_SOLVE_DAMPING) * pitchAxisError -
        //  axisCoupling * yawAxisError) /
        //     solveDeterminant,
        turret.pitchMotor);

    const float yawVelocity = turret.yawMotor.getChassisFrameVelocity();
    const float pitchVelocity = turret.pitchMotor.getChassisFrameVelocity();

    float yawOutput = 0.0f;
    if (yawHasPointingAuthority)
    {
        const float yawVelocitySetpoint = yawPositionPid.runController(yawError, yawVelocity, dt);
        yawOutput =
            yawVelocityPid.runControllerDerivateError(yawVelocitySetpoint - yawVelocity, dt);
    }
    else
    {
        yawPositionPid.reset();
        yawVelocityPid.reset();
        chassisFrameYawController.runController(
            dt,
            chassisFrameYawController.getSetpoint() + limitedYawInput);
        float chassisYawOutput = turret.yawMotor.getMotorOutput();
        if (stopAtMotorLimits(chassisYawOutput, turret.yawMotor))
        {
            chassisFrameYawController.setSetpoint(turret.yawMotor.getChassisFrameMeasuredAngle());
            turret.yawMotor.setMotorOutput(chassisYawOutput);
        }
    }
    const float pitchVelocitySetpoint =
        pitchPositionPid.runController(pitchError, pitchVelocity, dt);
    float pitchOutput =
        pitchVelocityPid.runControllerDerivateError(pitchVelocitySetpoint - pitchVelocity, dt);
    if (yawHasPointingAuthority)
    {
        if (stopAtMotorLimits(yawOutput, turret.yawMotor))
        {
            yawPositionPid.reset();
            yawVelocityPid.reset();
        }
        turret.yawMotor.setMotorOutput(yawOutput);
    }
    if (stopAtMotorLimits(pitchOutput, turret.pitchMotor))
    {
        pitchPositionPid.reset();
        pitchVelocityPid.reset();
    }

    turret.pitchMotor.setMotorOutput(pitchOutput);
}

void DroneTurretVectorCommand::runChassisFrameFallback(float yawInput, float pitchInput, float dt)
{
    if (!usingChassisFrameFallback)
    {
        resetChassisFrameFallback();
    }

    const float limitedYawInput = limitUserInputAtMotorLimits(yawInput, turret.yawMotor);
    const float limitedPitchInput = limitUserInputAtMotorLimits(pitchInput, turret.pitchMotor);

    chassisFrameYawController.runController(
        dt,
        chassisFrameYawController.getSetpoint() + limitedYawInput);
    chassisFramePitchController.runController(
        dt,
        chassisFramePitchController.getSetpoint() + limitedPitchInput);

    float yawOutput = turret.yawMotor.getMotorOutput();
    if (stopAtMotorLimits(yawOutput, turret.yawMotor))
    {
        chassisFrameYawController.setSetpoint(turret.yawMotor.getChassisFrameMeasuredAngle());
        turret.yawMotor.setMotorOutput(yawOutput);
    }

    float pitchOutput = turret.pitchMotor.getMotorOutput();
    if (stopAtMotorLimits(pitchOutput, turret.pitchMotor))
    {
        chassisFramePitchController.setSetpoint(turret.pitchMotor.getChassisFrameMeasuredAngle());
        turret.pitchMotor.setMotorOutput(pitchOutput);
    }
}

void DroneTurretVectorCommand::resetChassisFrameFallback()
{
    chassisFrameYawController.initialize();
    chassisFramePitchController.initialize();
    chassisFrameYawController.setSetpoint(turret.yawMotor.getChassisFrameMeasuredAngle());
    chassisFramePitchController.setSetpoint(turret.pitchMotor.getChassisFrameMeasuredAngle());
    usingChassisFrameFallback = true;
}

bool DroneTurretVectorCommand::isFinished() const
{
    return !turret.yawMotor.isOnline() && !turret.pitchMotor.isOnline();
}

void DroneTurretVectorCommand::end(bool)
{
    turret.yawMotor.setMotorOutput(0.0f);
    turret.pitchMotor.setMotorOutput(0.0f);
}

DroneTurretVectorCommand::Vector DroneTurretVectorCommand::getTurretForwardVectorWorldFrame() const
{
    const float q0 = turretImu.getQ0();
    const float q1 = turretImu.getQ1();
    const float q2 = turretImu.getQ2();
    const float q3 = turretImu.getQ3();

    return Vector(
        1.0f - 2.0f * (q2 * q2 + q3 * q3),
        2.0f * (q1 * q2 + q0 * q3),
        2.0f * (q1 * q3 - q0 * q2));
}

DroneTurretVectorCommand::Vector DroneTurretVectorCommand::getTurretPitchAxisWorldFrame() const
{
    const float q0 = turretImu.getQ0();
    const float q1 = turretImu.getQ1();
    const float q2 = turretImu.getQ2();
    const float q3 = turretImu.getQ3();

    return Vector(
        2.0f * (q1 * q2 - q0 * q3),
        1.0f - 2.0f * (q1 * q1 + q3 * q3),
        2.0f * (q2 * q3 + q0 * q1));
}

DroneTurretVectorCommand::Vector DroneTurretVectorCommand::getTurretYawAxisWorldFrame(
    Vector turretForwardWorldFrame,
    Vector turretPitchAxisWorldFrame) const
{
    const float pitchAngle = turret.pitchMotor.getChassisFrameMeasuredAngle().getWrappedValue();
    return normalizeOrFallback(
        rotateVector(turretForwardWorldFrame, turretPitchAxisWorldFrame, pitchAngle),
        lastYawAxisWorldFrame);
}

DroneTurretVectorCommand::Vector DroneTurretVectorCommand::rotateVector(
    Vector vector,
    Vector axis,
    float angle) const
{
    axis = normalizeOrFallback(axis, lastPitchInputAxisWorldFrame);
    const float c = cosf(angle);
    const float s = sinf(angle);
    return vector * c + axis.cross(vector) * s + axis * axis.dot(vector) * (1.0f - c);
}

DroneTurretVectorCommand::Vector DroneTurretVectorCommand::normalizeOrFallback(
    Vector vector,
    Vector fallback) const
{
    const float magnitude = vector.magnitude();
    if (magnitude < 1e-3f)
    {
        return fallback;
    }

    return vector / magnitude;
}

float DroneTurretVectorCommand::clampControllerError(
    float error,
    const aruwsrc::control::turret::TurretMotor& turretMotor) const
{
    error = tap::algorithms::limitVal(error, -MAX_CONTROLLER_ERROR, MAX_CONTROLLER_ERROR);

    if (!turretMotor.getConfig().limitMotorAngles)
    {
        return error;
    }

    const float angle = turretMotor.getChassisFrameMeasuredAngle().getUnwrappedValue();
    return tap::algorithms::limitVal(
        error,
        turretMotor.getConfig().minAngle - angle,
        turretMotor.getConfig().maxAngle - angle);
}

float DroneTurretVectorCommand::limitUserInputAtMotorLimits(
    float input,
    const aruwsrc::control::turret::TurretMotor& turretMotor) const
{
    if (!turretMotor.getConfig().limitMotorAngles)
    {
        return input;
    }

    const float angle = turretMotor.getChassisFrameMeasuredAngle().getUnwrappedValue();
    if ((angle <= turretMotor.getConfig().minAngle + LIMIT_INPUT_BUFFER && input < 0.0f) ||
        (angle >= turretMotor.getConfig().maxAngle - LIMIT_INPUT_BUFFER && input > 0.0f))
    {
        return 0.0f;
    }

    return input;
}

bool DroneTurretVectorCommand::stopAtMotorLimits(
    float& motorOutput,
    const aruwsrc::control::turret::TurretMotor& turretMotor) const
{
    if (!turretMotor.getConfig().limitMotorAngles)
    {
        return false;
    }

    const float angle = turretMotor.getChassisFrameMeasuredAngle().getUnwrappedValue();
    const float minAngle = turretMotor.getConfig().minAngle;
    const float maxAngle = turretMotor.getConfig().maxAngle;

    if ((angle <= minAngle && motorOutput < 0.0f) || (angle >= maxAngle && motorOutput > 0.0f))
    {
        motorOutput = 0.0f;
        return true;
    }

    return false;
}
}  // namespace aruwsrc::drone
