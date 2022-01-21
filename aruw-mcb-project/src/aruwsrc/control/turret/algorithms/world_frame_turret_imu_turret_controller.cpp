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

#include "../turret_subsystem.hpp"
#include "aruwsrc/drivers.hpp"

#include "turret_gravity_compensation.hpp"

namespace aruwsrc::control::turret::algorithms
{
/**
 * Transforms the specified `angleToTransform`, a yaw/pitch angle from the chassis frame to the
 * world frame.
 *
 * @note It is expected that the user wraps the value returned to be between [0, 360)
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
 * Transforms the specified `angleToTransform`, a yaw or pitch angle, from the world frame to
 * the chassis frame.
 *
 * @note It is expected that the user wraps the value returned to be between [0, 360)
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
 * A helper function for the `run*PidYawWorldFrameController` functions below. Updates the passed in
 * `turretSubsystem`'s desired chassis frame setpoint and the passed in `worldFrameYawSetpoint`'.
 * Performs necessary limiting of the `worldFrameYawSetpoint` based on the `turretSubsystem`'s
 * min/max yaw setpoints.
 *
 * @param[in] desiredSetpoint The new user-specified world frame turret yaw angle setpoint, in
 *      degrees.
 * @param[in] initChassisFrameImuAngle The initial chassis IMU angle, in degrees, measured from the
 *      chassis mounted IMU that is captured upon initialization of the chassis IMU world relative
 *      PID controller.
 * @param[in] currChassisFrameImuAngle The current chassis IMU angle, in degrees, measured from the
 *      chassis mounted IMU.
 * @param[out] worldFrameYawSetpoint The limited and wrapped world frame turret yaw setpoint, in
 *      degrees. Set to `desiredSetpoint` and then wrapped/limited as necessary.
 * @param[out] turretSubsystem The turret subsystem whose chassis relative turret yaw angle is
 *      updated by this function.
 */
static inline void updateYawWorldFrameSetpoint(
    const float desiredSetpoint,
    const float chassisFrameYaw,
    const float worldFrameYawAngle,
    tap::algorithms::ContiguousFloat *worldFrameYawSetpoint,
    tap::control::turret::TurretSubsystemInterface *turretSubsystem)
{
    worldFrameYawSetpoint->setValue(desiredSetpoint);

    // transform target angle from turret imu relative to chassis relative
    // to keep turret/command setpoints synchronized
    turretSubsystem->setYawSetpoint(transformWorldFrameValueToChassisFrame(
        chassisFrameYaw,
        worldFrameYawAngle,
        worldFrameYawSetpoint->getValue()));

    if (turretSubsystem->yawLimited())
    {
        // transform angle that is limited by subsystem to world relative again to run the
        // controller
        worldFrameYawSetpoint->setValue(transformChassisFrameToWorldFrame(
            chassisFrameYaw,
            worldFrameYawAngle,
            turretSubsystem->getYawSetpoint()));
    }
}

WorldFrameYawTurretImuCascadePidTurretController::WorldFrameYawTurretImuCascadePidTurretController(
    const aruwsrc::Drivers *drivers,
    TurretSubsystem *turretSubsystem,
    const tap::algorithms::SmoothPidConfig &posPidConfig,
    const tap::algorithms::SmoothPidConfig &velPidConfig)
    : TurretYawControllerInterface(turretSubsystem),
      drivers(drivers),
      positionPid(posPidConfig),
      velocityPid(velPidConfig),
      worldFrameSetpoint(0, 0, 360)
{
}

void WorldFrameYawTurretImuCascadePidTurretController::initialize()
{
    positionPid.reset();
    velocityPid.reset();

    // Capture initial target angle in chassis frame and transform to world frame.
    worldFrameSetpoint.setValue(transformChassisFrameToWorldFrame(
        turretSubsystem->getCurrentYawValue().getValue(),
        drivers->turretMCBCanComm.getYaw(),
        turretSubsystem->getYawSetpoint()));
}

void WorldFrameYawTurretImuCascadePidTurretController::runController(
    const uint32_t dt,
    const float desiredSetpoint)
{
    const float chassisFrameYaw = turretSubsystem->getCurrentYawValue().getValue();
    const float worldFrameYawAngle = drivers->turretMCBCanComm.getYaw();
    const float worldFrameYawVelocity = drivers->turretMCBCanComm.getYawVelocity();

    updateYawWorldFrameSetpoint(
        desiredSetpoint,
        chassisFrameYaw,
        worldFrameYawAngle,
        &worldFrameSetpoint,
        turretSubsystem);

    // position controller based on imu and yaw gimbal angle,
    // precisely, - (yawActual - worldFrameYawSetpoint), or more obvious,
    // worldFrameYawSetpoint - yawActual
    float positionControllerError = -worldFrameSetpoint.difference(worldFrameYawAngle);
    float positionPidOutput =
        positionPid.runController(positionControllerError, worldFrameYawVelocity, dt);

    float velocityControllerError = positionPidOutput - worldFrameYawVelocity;
    float velocityPidOutput = velocityPid.runControllerDerivateError(velocityControllerError, dt);

    turretSubsystem->setYawMotorOutput(velocityPidOutput);
}

float WorldFrameYawTurretImuCascadePidTurretController::getSetpoint() const
{
    return worldFrameSetpoint.getValue();
}

bool WorldFrameYawTurretImuCascadePidTurretController::isOnline() const
{
    return turretSubsystem->isOnline() && drivers->turretMCBCanComm.isConnected();
}

/**
 * A helper function for the `run*PidPitchWorldFrameController` functions below. Updates the passed
 * in `turretSubsystem`'s desired chassis frame setpoint and the passed in
 * `worldFramePitchSetpoint`'. Performs necessary limiting of the `worldFramePitchSetpoint` based on
 * the `turretSubsystem`'s min/max pitch setpoints.
 *
 * @param[in] desiredSetpoint The new user-specified world frame turret yaw angle setpoint, in
 *      degrees.
 * @param[in] initChassisFrameImuAngle The initial chassis IMU angle, in degrees, measured from the
 *      chassis mounted IMU that is captured upon initialization of the chassis IMU world relative
 *      PID controller.
 * @param[in] currChassisFrameImuAngle The current chassis IMU angle, in degrees, measured from the
 *      chassis mounted IMU.
 * @param[out] worldFrameYawSetpoint The limited and wrapped world frame turret yaw setpoint, in
 *      degrees. Set to `desiredSetpoint` and then wrapped/limited as necessary.
 * @param[out] turretSubsystem The turret subsystem whose chassis relative turret yaw angle is
 *      updated by this function.
 */
static inline void updatePitchWorldFrameSetpoint(
    const float desiredSetpoint,
    const float worldFramePitchAngle,
    tap::algorithms::ContiguousFloat *worldFramePitchSetpoint,
    tap::control::turret::TurretSubsystemInterface *turretSubsystem)
{
    worldFramePitchSetpoint->setValue(desiredSetpoint);

    // Project user desired setpoint that is in world relative to chassis relative
    // to limit the value
    turretSubsystem->setPitchSetpoint(transformWorldFrameValueToChassisFrame(
        turretSubsystem->getCurrentPitchValue().getValue(),
        worldFramePitchAngle,
        worldFramePitchSetpoint->getValue()));

    // Project angle limited by the tap::control::turret::TurretSubsystemInterface back to world
    // relative to use the value
    worldFramePitchSetpoint->setValue(transformChassisFrameToWorldFrame(
        turretSubsystem->getCurrentPitchValue().getValue(),
        worldFramePitchAngle,
        turretSubsystem->getPitchSetpoint()));
}

WorldFramePitchTurretImuCascadePidTurretController::
    WorldFramePitchTurretImuCascadePidTurretController(
        const aruwsrc::Drivers *drivers,
        TurretSubsystem *turretSubsystem,
        const tap::algorithms::SmoothPidConfig &posPidConfig,
        const tap::algorithms::SmoothPidConfig &velPidConfig)
    : TurretPitchControllerInterface(turretSubsystem),
      drivers(drivers),
      positionPid(posPidConfig),
      velocityPid(velPidConfig),
      worldFrameSetpoint(0, 0, 360)
{
}

void WorldFramePitchTurretImuCascadePidTurretController::initialize()
{
    positionPid.reset();
    velocityPid.reset();

    worldFrameSetpoint.setValue(transformChassisFrameToWorldFrame(
        turretSubsystem->getCurrentPitchValue().getValue(),
        drivers->turretMCBCanComm.getPitch(),
        turretSubsystem->getPitchSetpoint()));
}

void WorldFramePitchTurretImuCascadePidTurretController::runController(
    const uint32_t dt,
    const float desiredSetpoint)
{
    const float worldFramePitchAngle = drivers->turretMCBCanComm.getPitch();
    const float worldFramePitchVelocity = drivers->turretMCBCanComm.getPitchVelocity();

    updatePitchWorldFrameSetpoint(
        desiredSetpoint,
        worldFramePitchAngle,
        &worldFrameSetpoint,
        turretSubsystem);

    // Compute error between pitch target and reference angle
    float positionControllerError = -worldFrameSetpoint.difference(worldFramePitchAngle);
    float positionPidOutput =
        positionPid.runController(positionControllerError, worldFramePitchVelocity, dt);

    float velocityControllerError = positionPidOutput - worldFramePitchVelocity;
    float velocityPidOutput = velocityPid.runControllerDerivateError(velocityControllerError, dt);

    velocityPidOutput += computeGravitationalForceOffset(
        TurretSubsystem::TURRET_CG_X,
        TurretSubsystem::TURRET_CG_Z,
        turretSubsystem->getPitchAngleFromCenter(),
        TurretSubsystem::GRAVITY_COMPENSATION_SCALAR);

    turretSubsystem->setPitchMotorOutput(velocityPidOutput);
}

float WorldFramePitchTurretImuCascadePidTurretController::getSetpoint() const
{
    return worldFrameSetpoint.getValue();
}

bool WorldFramePitchTurretImuCascadePidTurretController::isOnline() const
{
    return turretSubsystem->isOnline() && drivers->turretMCBCanComm.isConnected();
}
}  // namespace aruwsrc::control::turret::algorithms
