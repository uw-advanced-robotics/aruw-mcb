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

#include "world_frame_turret_imu_turret_controller.hpp"

#include "tap/algorithms/contiguous_float.hpp"

#include "../constants/turret_constants.hpp"
#include "../turret_subsystem.hpp"
#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"

#include "turret_gravity_compensation.hpp"

using namespace tap::algorithms;

namespace aruwsrc::control::turret::algorithms
{
/**
 * Transforms the specified `angleToTransform`, a yaw/pitch angle (in radians) from the chassis
 * frame to the world frame.
 *
 * @note It is expected that the user wraps the value returned to be between [0, M_TWOPI)
 *      (or whatever range they require).
 *
 * @param[in] turretChassisFrameCurrAngle The current chassis relative (gimbal encoder) angle.
 * @param[in] turretWorldFrameCurrAngle The current world relative (turret IMU) angle,
 *      captured at the same time as `turretChassisFrameCurrAngle`.
 * @param[in] angleToTransform The angle to transform.
 * @return The transformed angle in the world frame.
 */
static inline float transformChassisFrameToWorldFrame(
    const float turretChassisFrameCurrAngle,
    const float turretWorldFrameCurrAngle,
    const float angleToTransform)
{
    return turretWorldFrameCurrAngle + (angleToTransform - turretChassisFrameCurrAngle);
}

/**
 * Transforms the specified `angleToTransform`, a yaw or pitch angle (in radians), from the world
 * frame to the chassis frame.
 *
 * @note It is expected that the user wraps the value returned to be between [0, M_TWOPI)
 *      (or whatever range they require).
 *
 * @param[in] turretChassisFrameCurrAngle The current chassis relative (gimbal encoder) angle.
 * @param[in] turretWorldFrameCurrAngle The current world relative (turret IMU) angle, captured
 *      at the same time as `turretChassisFrameCurrAngle`.
 * @param[in] angleToTransform The angle to transform.
 * @return The transformed angle in the chassis frame.
 */
static inline float transformWorldFrameValueToChassisFrame(
    const float turretChassisFrameCurrAngle,
    const float turretWorldFrameCurrAngle,
    const float angleToTransform)
{
    return turretChassisFrameCurrAngle + (angleToTransform - turretWorldFrameCurrAngle);
}

/**
 * Transforms the specified `angleToTransform`, a yaw angle (in radians), from the turret minor
 * frame to the turret major frame
 *
 * @note It is expected that the user wraps the value returned to be between [0, M_TWOPI)
 *      (or whatever range they require).
 *
 * @param[in] turretMinorAngle The current turret minor relative angle.
 * @param[in] turretMajorAngle The current turret major angle, captured
 *      at the same time as `turretMinorAngle`.
 * @param[in] angleToTransform The angle to transform.
 * @return The transformed angle in the turret major frame.
 */
static inline float transformMinorToMajor(
    const float turretMinorAngle,
    const float turretMajorAngle,
    const float angleToTransform)
{
    return turretMajorAngle - turretMinorAngle + angleToTransform;
}

/**
 * Transforms the specified `angleToTransform`, a yaw angle (in radians), from the turret major
 * frame to the chassis frame
 *
 * @note It is expected that the user wraps the value returned to be between [0, M_TWOPI)
 *      (or whatever range they require).
 *
 * @param[in] turretMajorAngle The current turret major relative angle.
 * @param[in] chassisAngle The current chassis relative (gimbal encoder) angle, captured
 *      at the same time as `turretMinorAngle`.
 * @param[in] angleToTransform The angle to transform.
 * @return The transformed angle in the chassis frame.
 */
static inline float transformMajorToChassis(
    const float turretMajorAngle,
    const float chassisAngle,
    const float angleToTransform)
{
    return chassisAngle - turretMajorAngle + angleToTransform;
}

/**
 * Initializes a world frame cascade PID turret controller
 *
 * @param[in] controllerToInitialize The TurretControllerInterface in question being initialized.
 * @param[in] worldFrameMeasurement The measured world frame angle, in radians, not expected to be
 * normalized.
 * @param[out] turretMotor The turret motor that will be controlled by the passed in
 * controllerToInitialize.
 * @param[out] positionPid Position PID controller.
 * @param[out] velocityPid Velocity PID controller.
 * @param[out] worldFrameSetpoint World frame angle setpoint that will be set to the current
 * turretMotor's setpoint.
 */
static inline void initializeWorldFrameTurretImuController(
    const TurretControllerInterface *controllerToInitialize,
    const float worldFrameMeasurement,
    TurretMotor &turretMotor,
    tap::algorithms::SmoothPid &positionPid,
    tap::algorithms::SmoothPid &velocityPid,
    float &worldFrameSetpoint)
{
    if (turretMotor.getTurretController() != controllerToInitialize)
    {
        positionPid.reset();
        velocityPid.reset();

        worldFrameSetpoint = transformChassisFrameToWorldFrame(
            turretMotor.getChassisFrameUnwrappedMeasuredAngle(),
            worldFrameMeasurement,
            turretMotor.getChassisFrameSetpoint());

        turretMotor.attachTurretController(controllerToInitialize);
    }
}

/**
 * A helper function for the `run*PidYawWorldFrameController` functions below. Updates the passed in
 * `turretMotor`'s desired chassis frame setpoint and the passed in `worldFrameSetpoint`'.
 * Performs necessary limiting of the `worldFrameSetpoint` based on the `turretMotor`'s
 * min/max setpoints.
 *
 * @param[in] desiredSetpoint The new user-specified world frame turret motor angle setpoint, in
 * radians.
 * @param[in] chassisFrameMeasurement The chassis frame motor angle, in radians, measured by the
 * motor's encoder.
 * @param[in] worldFrameMeasurement The current chassis IMU angle, in radians, measured from the
 * chassis mounted IMU.
 * @param[out] worldFrameSetpoint The limited and wrapped world frame turret motor setpoint, in
 * radians. Set to `desiredSetpoint` and then wrapped/limited as necessary.
 * @param[out] turretMotor The turret subsystem whose chassis relative turret motor angle is
 * updated by this function.
 */

// TODO: don't use n
static inline void updateWorldFrameSetpoint(
    const float desiredSetpoint,
    const float chassisFrameMeasurement,
    const float worldFrameMeasurement,
    float &worldFrameSetpoint,
    TurretMotor &turretMotor)
{
    worldFrameSetpoint = desiredSetpoint;

    // transform target angle from turret imu relative to chassis relative
    // to keep turret/command setpoints synchronized

    // desired is in world frame, major = desired + major yaw
    // float desiredSetPointMajorRel = desiredSetpoint - majorFrameMeasurement;
    // turretMotor.setChassisFrameSetpoint(desiredSetPointMajorRel);

    turretMotor.setChassisFrameSetpoint(transformWorldFrameValueToChassisFrame(
        chassisFrameMeasurement,
        worldFrameMeasurement,
        worldFrameSetpoint));

    // @TODO: limiting
    // does not actually perform any motor limiting in terms of preventing the turret from breaking
    // all it does is read the limited value from the turretMotor, translating that back
    // into the world frame
    if (turretMotor.getConfig().limitMotorAngles)
    {
        // transform angle that is limited by subsystem to world relative again to run the
        // controller
        worldFrameSetpoint = transformChassisFrameToWorldFrame(
            chassisFrameMeasurement,
            worldFrameMeasurement,
            turretMotor.getChassisFrameSetpoint());
    }
}

/**
 * Runs a world frame cascade (position -> velocity) PID controller.
 *
 * @param[in] worldFrameAngleSetpoint World frame angle setpoint, not required to be normalized, in
 * radians.
 * @param[in] worldFrameAngleMeasurement World frame angle measurement, not required to be
 * normalized, in radians.
 * @param[in] worldFrameVelocityMeasured World frame angular velocity measurement, in
 * radians/second.
 * @param[in] dt Time change since this function was last called, in ms.
 * @param[in] turretMotor TurretMotor associated with the angles being measured.
 * @param[out] positionPid Position PID controller.
 * @param[out] velocityPid Velocity PID controller.
 * @return desired PID output from running the position -> velocity cascade controller
 */

// TODO: only use for reference
static inline float runWorldFrameTurretImuController(
    const float worldFrameAngleSetpoint,
    const float worldFrameAngleMeasurement,
    const float worldFrameVelocityMeasured,
    const uint32_t dt,
    const TurretMotor &turretMotor,
    tap::algorithms::SmoothPid &positionPid,
    tap::algorithms::SmoothPid &velocityPid)
{
    const float positionControllerError =
        turretMotor.getValidMinError(worldFrameAngleSetpoint, worldFrameAngleMeasurement);
    const float positionPidOutput =
        positionPid.runController(positionControllerError, worldFrameVelocityMeasured, dt);

    const float velocityControllerError = positionPidOutput - worldFrameVelocityMeasured;
    const float velocityPidOutput =
        velocityPid.runControllerDerivateError(velocityControllerError, dt);

    return velocityPidOutput;
}

WorldFrameYawTurretImuCascadePidTurretController::WorldFrameYawTurretImuCascadePidTurretController(
    const aruwsrc::can::TurretMCBCanComm &turretMCBCanComm,
    TurretMotor &yawMotor,
    const TurretMotor &majorMotor,
    tap::algorithms::SmoothPid &positionPid,
    tap::algorithms::SmoothPid &velocityPid)
    : TurretYawControllerInterface(yawMotor),
      turretMCBCanComm(turretMCBCanComm),
      positionPid(positionPid),
      velocityPid(velocityPid),
      worldFrameSetpoint(0),
      majorMotor(majorMotor)
{
}

void WorldFrameYawTurretImuCascadePidTurretController::initialize()
{
    initializeWorldFrameTurretImuController(
        this,
        turretMCBCanComm.getYawUnwrapped(),
        turretMotor,
        positionPid,
        velocityPid,
        worldFrameSetpoint);
}

// float positionPidOutput = 0;

void WorldFrameYawTurretImuCascadePidTurretController::runController(
    const uint32_t dt,
    const float desiredSetpoint)  // desired yaw setpoint in the world frame
{

    const float idealLocalSetpoint = desiredSetpoint + turretMotor.getChassisFrameUnwrappedMeasuredAngle() - turretMCBCanComm.getYawUnwrapped();
    turretMotor.setChassisFrameSetpoint(idealLocalSetpoint);
    const float realLocalSetpoint = turretMotor.getChassisFrameSetpoint();

    this->worldFrameSetpoint = realLocalSetpoint - turretMotor.getChassisFrameUnwrappedMeasuredAngle() + turretMCBCanComm.getYawUnwrapped();

    // use PID to compute how to get to where we want to go
    const float positionControllerError = turretMotor.getValidMinError(
        this->worldFrameSetpoint,
        turretMCBCanComm.getYawUnwrapped());

    const float positionPidOutput =
        positionPid.runController(positionControllerError, turretMCBCanComm.getYawVelocity(), dt);

    const float velocityControllerError = positionPidOutput - turretMCBCanComm.getYawVelocity();
    const float velocityPidOutput =
        velocityPid.runControllerDerivateError(velocityControllerError, dt);

    turretMotor.setMotorOutput(velocityPidOutput);
}

// NOTE: this is out of date
void WorldFrameYawTurretImuCascadePidTurretController::setSetpoint(float desiredSetpoint)
{
    // const float chassisFrameYaw = turretMotor.getChassisFrameUnwrappedMeasuredAngle();
    // const float worldFrameYawAngle = turretMCBCanComm.getYawUnwrapped();

    const float chassisToMajor = majorMotor.getChassisFrameUnwrappedMeasuredAngle();

    // convert desiredsetpoint to major frame
    const float desiredSetPointMajorRel = desiredSetpoint + chassisToMajor;

    // where we want the turret to point in world frame
    worldFrameSetpoint = desiredSetpoint;

    // indicate this is where we want to go
    turretMotor.setChassisFrameSetpoint(desiredSetPointMajorRel);

    // updateWorldFrameSetpoint(
    //     desiredSetpoint,
    //     chassisFrameYaw,
    //     worldFrameYawAngle,
    //     worldFrameSetpoint,
    //     turretMotor);
}

float WorldFrameYawTurretImuCascadePidTurretController::getSetpoint() const
{
    return worldFrameSetpoint;
}

float WorldFrameYawTurretImuCascadePidTurretController::getMeasurement() const
{
    return turretMCBCanComm.getYawUnwrapped();
}

bool WorldFrameYawTurretImuCascadePidTurretController::isOnline() const
{
    return turretMotor.isOnline() && turretMCBCanComm.isConnected();
}

// TODO: don't use this
float WorldFrameYawTurretImuCascadePidTurretController::convertControllerAngleToChassisFrame(
    float controllerFrameAngle) const
{
    const float worldFrameYawAngle = turretMCBCanComm.getYawUnwrapped();

    return transformWorldFrameValueToChassisFrame(
        turretMotor.getChassisFrameUnwrappedMeasuredAngle(),
        worldFrameYawAngle,
        controllerFrameAngle);
}

// TODO: don't use this
float WorldFrameYawTurretImuCascadePidTurretController::convertChassisAngleToControllerFrame(
    float chassisFrameAngle) const
{
    const float worldFrameYawAngle = turretMCBCanComm.getYawUnwrapped();

    return transformChassisFrameToWorldFrame(
        turretMotor.getChassisFrameUnwrappedMeasuredAngle(),
        worldFrameYawAngle,
        chassisFrameAngle);
}

// ??????
WorldFramePitchTurretImuCascadePidTurretController::
    WorldFramePitchTurretImuCascadePidTurretController(
        const aruwsrc::can::TurretMCBCanComm &turretMCBCanComm,
        TurretMotor &turretMotor,
        tap::algorithms::SmoothPid &positionPid,
        tap::algorithms::SmoothPid &velocityPid)
    : TurretPitchControllerInterface(turretMotor),
      turretMCBCanComm(turretMCBCanComm),
      positionPid(positionPid),
      velocityPid(velocityPid),
      worldFrameSetpoint(0)
{
}

void WorldFramePitchTurretImuCascadePidTurretController::initialize()
{
    initializeWorldFrameTurretImuController(
        this,
        turretMCBCanComm.getPitchUnwrapped(),
        turretMotor,
        positionPid,
        velocityPid,
        worldFrameSetpoint);
}

void WorldFramePitchTurretImuCascadePidTurretController::runController(
    const uint32_t dt,
    const float desiredSetpoint)
{
    const float chassisFramePitch = turretMotor.getChassisFrameUnwrappedMeasuredAngle();
    const float worldFramePitchAngle = turretMCBCanComm.getPitchUnwrapped();
    const float worldFramePitchVelocity = turretMCBCanComm.getPitchVelocity();

    updateWorldFrameSetpoint(
        desiredSetpoint,
        chassisFramePitch,
        worldFramePitchAngle,
        worldFrameSetpoint,
        turretMotor);

    float pidOut = runWorldFrameTurretImuController(
        worldFrameSetpoint,
        worldFramePitchAngle,
        worldFramePitchVelocity,
        dt,
        turretMotor,
        positionPid,
        velocityPid);

    pidOut += computeGravitationalForceOffset(
        TURRET_CG_X,
        TURRET_CG_Z,
        -turretMotor.getAngleFromCenter(),
        GRAVITY_COMPENSATION_SCALAR);

    turretMotor.setMotorOutput(pidOut);
}

void WorldFramePitchTurretImuCascadePidTurretController::setSetpoint(float desiredSetpoint)
{
    const float chassisFramePitch = turretMotor.getChassisFrameUnwrappedMeasuredAngle();
    const float worldFramePitchAngle = turretMCBCanComm.getPitchUnwrapped();

    updateWorldFrameSetpoint(
        desiredSetpoint,
        chassisFramePitch,
        worldFramePitchAngle,
        worldFrameSetpoint,
        turretMotor);
}

float WorldFramePitchTurretImuCascadePidTurretController::getSetpoint() const
{
    return worldFrameSetpoint;
}

float WorldFramePitchTurretImuCascadePidTurretController::getMeasurement() const
{
    return turretMCBCanComm.getPitchUnwrapped();
}

bool WorldFramePitchTurretImuCascadePidTurretController::isOnline() const
{
    return turretMotor.isOnline() && turretMCBCanComm.isConnected();
}

float WorldFramePitchTurretImuCascadePidTurretController::convertControllerAngleToChassisFrame(
    float controllerFrameAngle) const
{
    const float worldFramePitchAngle = turretMCBCanComm.getPitchUnwrapped();

    return transformWorldFrameValueToChassisFrame(
        turretMotor.getChassisFrameUnwrappedMeasuredAngle(),
        worldFramePitchAngle,
        controllerFrameAngle);
}

float WorldFramePitchTurretImuCascadePidTurretController::convertChassisAngleToControllerFrame(
    float chassisFrameAngle) const
{
    const float worldFramePitchAngle = turretMCBCanComm.getPitchUnwrapped();

    return transformChassisFrameToWorldFrame(
        turretMotor.getChassisFrameUnwrappedMeasuredAngle(),
        worldFramePitchAngle,
        chassisFrameAngle);
}
}  // namespace aruwsrc::control::turret::algorithms
