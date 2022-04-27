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

#include "../constants/turret_constants.hpp"
#include "../turret_subsystem.hpp"
#include "aruwsrc/drivers.hpp"

#include "turret_gravity_compensation.hpp"

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
 * A helper function for the `run*PidYawWorldFrameController` functions below. Updates the passed in
 * `yawMotor`'s desired chassis frame setpoint and the passed in `worldFrameYawSetpoint`'.
 * Performs necessary limiting of the `worldFrameYawSetpoint` based on the `yawMotor`'s
 * min/max yaw setpoints.
 *
 * @param[in] desiredSetpoint The new user-specified world frame turret yaw angle setpoint, in
 *      degrees.
 * @param[in] chassisFrameYaw The chassis frame yaw angle, in radians, measured by the motor's
 *      encoder.
 * @param[in] worldFrameYawAngle The current chassis IMU angle, in radians, measured from the
 *      chassis mounted IMU.
 * @param[out] worldFrameYawSetpoint The limited and wrapped world frame turret yaw setpoint, in
 *      radians. Set to `desiredSetpoint` and then wrapped/limited as necessary.
 * @param[out] yawMotor The turret subsystem whose chassis relative turret yaw angle is
 *      updated by this function.
 */
static inline void updateYawWorldFrameSetpoint(
    const float desiredSetpoint,
    const float chassisFrameYaw,
    const float worldFrameYawAngle,
    tap::algorithms::ContiguousFloat *worldFrameYawSetpoint,
    TurretMotor *yawMotor)
{
    worldFrameYawSetpoint->setValue(desiredSetpoint);

    // transform target angle from turret imu relative to chassis relative
    // to keep turret/command setpoints synchronized

    yawMotor->setChassisFrameSetpoint(transformWorldFrameValueToChassisFrame(
        chassisFrameYaw,
        worldFrameYawAngle,
        worldFrameYawSetpoint->getValue()));

    if (yawMotor->getConfig().limitMotorAngles)
    {
        // transform angle that is limited by subsystem to world relative again to run the
        // controller
        worldFrameYawSetpoint->setValue(transformChassisFrameToWorldFrame(
            chassisFrameYaw,
            worldFrameYawAngle,
            yawMotor->getChassisFrameSetpoint()));
    }
}

WorldFrameYawTurretImuCascadePidTurretController::WorldFrameYawTurretImuCascadePidTurretController(
    const aruwsrc::Drivers *drivers,
    TurretMotor *yawMotor,
    const tap::algorithms::SmoothPidConfig &posPidConfig,
    const tap::algorithms::SmoothPidConfig &velPidConfig)
    : TurretYawControllerInterface(yawMotor),
      drivers(drivers),
      positionPid(posPidConfig),
      velocityPid(velPidConfig),
      worldFrameSetpoint(0, 0, M_TWOPI)
{
}

void WorldFrameYawTurretImuCascadePidTurretController::initialize()
{
    if (this != turretMotor->getTurretController())
    {
        positionPid.reset();
        velocityPid.reset();

        // Capture initial target angle in chassis frame and transform to world frame.
        worldFrameSetpoint.setValue(transformChassisFrameToWorldFrame(
            turretMotor->getChassisFrameMeasuredAngle().getValue(),
            drivers->turretMCBCanComm.getYaw(),
            turretMotor->getChassisFrameSetpoint()));

        turretMotor->attachTurretController(this);
    }
}

void WorldFrameYawTurretImuCascadePidTurretController::runController(
    const uint32_t dt,
    const float desiredSetpoint)
{
    const float chassisFrameYaw = turretMotor->getChassisFrameMeasuredAngle().getValue();
    const float worldFrameYawAngle = drivers->turretMCBCanComm.getYaw();
    const float worldFrameYawVelocity = drivers->turretMCBCanComm.getYawVelocity();

    updateYawWorldFrameSetpoint(
        desiredSetpoint,
        chassisFrameYaw,
        worldFrameYawAngle,
        &worldFrameSetpoint,
        turretMotor);

    // position controller based on imu and yaw gimbal angle,
    // precisely, - (yawActual - worldFrameYawSetpoint), or more obvious,
    // worldFrameYawSetpoint - yawActual
    float positionControllerError = -worldFrameSetpoint.difference(worldFrameYawAngle);
    float positionPidOutput =
        positionPid.runController(positionControllerError, worldFrameYawVelocity, dt);

    float velocityControllerError = positionPidOutput - worldFrameYawVelocity;
    float velocityPidOutput = velocityPid.runControllerDerivateError(velocityControllerError, dt);

    turretMotor->setMotorOutput(velocityPidOutput);
}

void WorldFrameYawTurretImuCascadePidTurretController::setSetpoint(float desiredSetpoint)
{
    const float chassisFrameYaw = turretSubsystem->getCurrentYawValue().getValue();
    const float worldFrameYawAngle = drivers->turretMCBCanComm.getYaw();

    updateYawWorldFrameSetpoint(
        desiredSetpoint,
        chassisFrameYaw,
        worldFrameYawAngle,
        &worldFrameSetpoint,
        turretSubsystem);
}

float WorldFrameYawTurretImuCascadePidTurretController::getSetpoint() const
{
    return worldFrameSetpoint.getValue();
}

bool WorldFrameYawTurretImuCascadePidTurretController::isOnline() const
{
    return turretMotor->isOnline() && drivers->turretMCBCanComm.isConnected();
}

HeroTurretImuCascadePidTurretController::HeroTurretImuCascadePidTurretController(
    const aruwsrc::Drivers *drivers,
    TurretMotor *turretMotor,
    const tap::algorithms::SmoothPidConfig &posPidConfig,
    const tap::algorithms::FuzzyPDConfig &fuzzyPidConfig,
    const tap::algorithms::SmoothPidConfig &velPidConfig)
    : TurretYawControllerInterface(turretMotor),
      drivers(drivers),
      positionPid(fuzzyPidConfig, posPidConfig),
      velocityPid(velPidConfig),
      worldFrameSetpoint(0, 0, M_TWOPI)
{
}

void HeroTurretImuCascadePidTurretController::initialize()
{
    if (this != turretMotor->getTurretController())
    {
        positionPid.reset();
        velocityPid.reset();

        // Capture initial target angle in chassis frame and transform to world frame.
        worldFrameSetpoint.setValue(transformChassisFrameToWorldFrame(
            turretMotor->getChassisFrameMeasuredAngle().getValue(),
            drivers->turretMCBCanComm.getYaw(),
            turretMotor->getChassisFrameSetpoint()));

        turretMotor->attachTurretController(this);
    }
}

void HeroTurretImuCascadePidTurretController::runController(
    const uint32_t dt,
    const float desiredSetpoint)
{
    const float chassisFrameYaw = turretMotor->getChassisFrameMeasuredAngle().getValue();
    const float worldFrameYawAngle = drivers->turretMCBCanComm.getYaw();
    const float worldFrameYawVelocity = drivers->turretMCBCanComm.getYawVelocity();

    updateYawWorldFrameSetpoint(
        desiredSetpoint,
        chassisFrameYaw,
        worldFrameYawAngle,
        &worldFrameSetpoint,
        turretMotor);

    // position controller based on imu and yaw gimbal angle,
    // precisely, - (yawActual - worldFrameYawSetpoint), or more obvious,
    // worldFrameYawSetpoint - yawActual
    float positionControllerError = -worldFrameSetpoint.difference(worldFrameYawAngle);
    float positionPidOutput =
        positionPid.runController(positionControllerError, worldFrameYawVelocity, dt);

    float velocityControllerError = positionPidOutput - worldFrameYawVelocity;
    float velocityPidOutput = velocityPid.runControllerDerivateError(velocityControllerError, dt);

    turretMotor->setMotorOutput(velocityPidOutput);
}

void HeroTurretImuCascadePidTurretController::setSetpoint(float desiredSetpoint)
{
    const float chassisFrameYaw = turretSubsystem->getCurrentYawValue().getValue();
    const float worldFrameYawAngle = drivers->turretMCBCanComm.getYaw();

    updateYawWorldFrameSetpoint(
        desiredSetpoint,
        chassisFrameYaw,
        worldFrameYawAngle,
        &worldFrameSetpoint,
        turretSubsystem);
}

float HeroTurretImuCascadePidTurretController::getSetpoint() const
{
    return worldFrameSetpoint.getValue();
}

bool HeroTurretImuCascadePidTurretController::isOnline() const
{
    return turretMotor->isOnline() && drivers->turretMCBCanComm.isConnected();
}

/**
 * A helper function for the `run*PidPitchWorldFrameController` functions below. Updates the passed
 * in `pitchMotor`'s desired chassis frame setpoint and the passed in
 * `worldFramePitchSetpoint`'. Performs necessary limiting of the `worldFramePitchSetpoint` based on
 * the `pitchMotor`'s min/max pitch setpoints.
 *
 * @param[in] desiredSetpoint The new user-specified world frame turret pitch angle setpoint, in
 *      radidans.
 * @param[in] worldFramePitchAngle The world frame pitch angle, as measured by an IMU on the turret,
 *      in radians.
 * @param[in] worldFramePitchSetpoint The limited and wrapped world frame turret pitch setpoint, in
 *      radians. Set to `desiredSetpoint` and then wrapped/limited as necessary.
 * @param[out] pitchMotor The turret motor whose chassis relative turret pitch angle is
 *      updated by this function.
 */
static inline void updatePitchWorldFrameSetpoint(
    const float desiredSetpoint,
    const float worldFramePitchAngle,
    tap::algorithms::ContiguousFloat *worldFramePitchSetpoint,
    TurretMotor *pitchMotor)
{
    worldFramePitchSetpoint->setValue(desiredSetpoint);

    // Project user desired setpoint that is in world relative to chassis relative
    // to limit the value
    pitchMotor->setChassisFrameSetpoint(transformWorldFrameValueToChassisFrame(
        pitchMotor->getChassisFrameMeasuredAngle().getValue(),
        worldFramePitchAngle,
        worldFramePitchSetpoint->getValue()));

    // Project angle limited by the TurretMotor back to world
    // relative to use the value
    worldFramePitchSetpoint->setValue(transformChassisFrameToWorldFrame(
        pitchMotor->getChassisFrameMeasuredAngle().getValue(),
        worldFramePitchAngle,
        pitchMotor->getChassisFrameSetpoint()));
}

WorldFramePitchTurretImuCascadePidTurretController::
    WorldFramePitchTurretImuCascadePidTurretController(
        const aruwsrc::Drivers *drivers,
        TurretMotor *turretMotor,
        const tap::algorithms::SmoothPidConfig &posPidConfig,
        const tap::algorithms::SmoothPidConfig &velPidConfig)
    : TurretPitchControllerInterface(turretMotor),
      drivers(drivers),
      positionPid(posPidConfig),
      velocityPid(velPidConfig),
      worldFrameSetpoint(0, 0, M_TWOPI)
{
}

void WorldFramePitchTurretImuCascadePidTurretController::initialize()
{
    if (turretMotor->getTurretController() != this)
    {
        positionPid.reset();
        velocityPid.reset();

        worldFrameSetpoint.setValue(transformChassisFrameToWorldFrame(
            turretMotor->getChassisFrameMeasuredAngle().getValue(),
            drivers->turretMCBCanComm.getPitch(),
            turretMotor->getChassisFrameSetpoint()));

        turretMotor->attachTurretController(this);
    }
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
        turretMotor);

    // Compute error between pitch target and reference angle
    float positionControllerError = -worldFrameSetpoint.difference(worldFramePitchAngle);
    float positionPidOutput =
        positionPid.runController(positionControllerError, worldFramePitchVelocity, dt);

    float velocityControllerError = positionPidOutput - worldFramePitchVelocity;
    float velocityPidOutput = velocityPid.runControllerDerivateError(velocityControllerError, dt);

    velocityPidOutput += computeGravitationalForceOffset(
        TURRET_CG_X,
        TURRET_CG_Z,
        -turretMotor->getAngleFromCenter(),
        GRAVITY_COMPENSATION_SCALAR);

    turretMotor->setMotorOutput(velocityPidOutput);
}

void WorldFramePitchTurretImuCascadePidTurretController::setSetpoint(float desiredSetpoint)
{
    const float worldFramePitchAngle = drivers->turretMCBCanComm.getPitch();

    updatePitchWorldFrameSetpoint(
        desiredSetpoint,
        worldFramePitchAngle,
        &worldFrameSetpoint,
        turretSubsystem);
}

float WorldFramePitchTurretImuCascadePidTurretController::getSetpoint() const
{
    return worldFrameSetpoint.getValue();
}

bool WorldFramePitchTurretImuCascadePidTurretController::isOnline() const
{
    return turretMotor->isOnline() && drivers->turretMCBCanComm.isConnected();
}
}  // namespace aruwsrc::control::turret::algorithms
