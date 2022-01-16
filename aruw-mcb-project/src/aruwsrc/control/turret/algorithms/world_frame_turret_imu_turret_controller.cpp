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

#include "aruwsrc/drivers.hpp"

#include "turret_gravity_compensation.hpp"

namespace aruwsrc::control::turret
{
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
    TurretSubsystemInterface *turretSubsystem)
{
    worldFrameYawSetpoint->setValue(desiredSetpoint);

    // transform target angle from turret imu relative to chassis relative
    // to keep turret/command setpoints synchronized
    turretSubsystem->setYawSetpoint(
        WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(
            chassisFrameYaw,
            worldFrameYawAngle,
            worldFrameYawSetpoint->getValue()));

    if (turretSubsystem->yawLimited())
    {
        // transform angle that is limited by subsystem to world relative again to run the
        // controller
        worldFrameYawSetpoint->setValue(
            WorldFrameTurretImuTurretController::transformChassisFrameToWorldFrame(
                chassisFrameYaw,
                worldFrameYawAngle,
                turretSubsystem->getYawSetpoint()));
    }
}

void WorldFrameTurretImuTurretController::runYawPidController(
    const aruwsrc::Drivers &drivers,
    const uint32_t dt,
    const float desiredSetpoint,
    tap::algorithms::ContiguousFloat *worldFrameYawSetpoint,
    tap::algorithms::SmoothPid *yawPid,
    TurretSubsystemInterface *turretSubsystem)
{
    const float chassisFrameYaw = turretSubsystem->getCurrentYawValue().getValue();
    const float worldFrameYawAngle = drivers.turretMCBCanComm.getYaw();
    const float worldFrameYawVelocity = drivers.turretMCBCanComm.getYawVelocity();

    updateYawWorldFrameSetpoint(
        desiredSetpoint,
        chassisFrameYaw,
        worldFrameYawAngle,
        worldFrameYawSetpoint,
        turretSubsystem);

    // position controller based on imu and yaw gimbal angle,
    // precisely, - (yawActual - worldFrameYawSetpoint), or more obvious,
    // worldFrameYawSetpoint - yawActual
    float positionControllerError = -worldFrameYawSetpoint->difference(worldFrameYawAngle);
    float pidOutput = yawPid->runController(positionControllerError, worldFrameYawVelocity, dt);

    turretSubsystem->setYawMotorOutput(pidOutput);
}

void WorldFrameTurretImuTurretController::runYawCascadePidController(
    const aruwsrc::Drivers &drivers,
    const uint32_t dt,
    const float desiredSetpoint,
    tap::algorithms::ContiguousFloat *worldFrameYawSetpoint,
    tap::algorithms::SmoothPid *yawPositionPid,
    tap::algorithms::SmoothPid *yawVelocityPid,
    TurretSubsystemInterface *turretSubsystem)
{
    const float chassisFrameYaw = turretSubsystem->getCurrentYawValue().getValue();
    const float worldFrameYawAngle = drivers.turretMCBCanComm.getYaw();
    const float worldFrameYawVelocity = drivers.turretMCBCanComm.getYawVelocity();

    updateYawWorldFrameSetpoint(
        desiredSetpoint,
        chassisFrameYaw,
        worldFrameYawAngle,
        worldFrameYawSetpoint,
        turretSubsystem);

    // position controller based on imu and yaw gimbal angle,
    // precisely, - (yawActual - worldFrameYawSetpoint), or more obvious,
    // worldFrameYawSetpoint - yawActual
    float positionControllerError = -worldFrameYawSetpoint->difference(worldFrameYawAngle);
    float positionPidOutput =
        yawPositionPid->runController(positionControllerError, worldFrameYawVelocity, dt);

    float velocityControllerError = positionPidOutput - worldFrameYawVelocity;
    float velocityPidOutput =
        yawVelocityPid->runControllerDerivateError(velocityControllerError, dt);

    turretSubsystem->setYawMotorOutput(velocityPidOutput);
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
    TurretSubsystemInterface *turretSubsystem)
{
    worldFramePitchSetpoint->setValue(desiredSetpoint);

    // Project user desired setpoint that is in world relative to chassis relative
    // to limit the value
    turretSubsystem->setPitchSetpoint(
        WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(
            turretSubsystem->getCurrentPitchValue().getValue(),
            worldFramePitchAngle,
            worldFramePitchSetpoint->getValue()));

    // Project angle limited by the aruwsrc::control::turret::TurretSubsystemInterface back to world
    // relative to use the value
    worldFramePitchSetpoint->setValue(
        WorldFrameTurretImuTurretController::transformChassisFrameToWorldFrame(
            turretSubsystem->getCurrentPitchValue().getValue(),
            worldFramePitchAngle,
            turretSubsystem->getPitchSetpoint()));
}

void WorldFrameTurretImuTurretController::runPitchPidController(
    const aruwsrc::Drivers &drivers,
    const uint32_t dt,
    const float desiredSetpoint,
    const float turretCGX,
    const float turretCGZ,
    const float gravityCompensationMotorOutputMax,
    tap::algorithms::ContiguousFloat *worldFramePitchSetpoint,
    tap::algorithms::SmoothPid *pitchPid,
    TurretSubsystemInterface *turretSubsystem)
{
    const float turretWorldFramePitchAngle = drivers.turretMCBCanComm.getPitch();
    const float turretWorldFramePitchVelocity = drivers.turretMCBCanComm.getPitchVelocity();

    updatePitchWorldFrameSetpoint(
        desiredSetpoint,
        turretWorldFramePitchAngle,
        worldFramePitchSetpoint,
        turretSubsystem);

    // Compute error between pitch target and reference angle
    float positionControllerError =
        -worldFramePitchSetpoint->difference(turretWorldFramePitchAngle);

    float pidOutput =
        pitchPid->runController(positionControllerError, turretWorldFramePitchVelocity, dt);

    pidOutput += computeGravitationalForceOffset(
        turretCGX,
        turretCGZ,
        turretSubsystem->getPitchAngleFromCenter(),
        gravityCompensationMotorOutputMax);

    turretSubsystem->setPitchMotorOutput(pidOutput);
}

void WorldFrameTurretImuTurretController::runPitchCascadePidController(
    const aruwsrc::Drivers &drivers,
    const uint32_t dt,
    const float desiredSetpoint,
    const float turretCGX,
    const float turretCGZ,
    const float gravityCompensationMotorOutputMax,
    tap::algorithms::ContiguousFloat *worldFramePitchSetpoint,
    tap::algorithms::SmoothPid *pitchPositionPid,
    tap::algorithms::SmoothPid *pitchVelocityPid,
    TurretSubsystemInterface *turretSubsystem)
{
    const float worldFramePitchAngle = drivers.turretMCBCanComm.getPitch();
    const float worldFramePitchVelocity = drivers.turretMCBCanComm.getPitchVelocity();

    updatePitchWorldFrameSetpoint(
        desiredSetpoint,
        worldFramePitchAngle,
        worldFramePitchSetpoint,
        turretSubsystem);

    // Compute error between pitch target and reference angle
    float positionControllerError = -worldFramePitchSetpoint->difference(worldFramePitchAngle);
    float positionPidOutput =
        pitchPositionPid->runController(positionControllerError, worldFramePitchVelocity, dt);

    float velocityControllerError = positionPidOutput - worldFramePitchVelocity;
    float velocityPidOutput =
        pitchVelocityPid->runControllerDerivateError(velocityControllerError, dt);

    velocityPidOutput += computeGravitationalForceOffset(
        turretCGX,
        turretCGZ,
        turretSubsystem->getPitchAngleFromCenter(),
        gravityCompensationMotorOutputMax);

    turretSubsystem->setPitchMotorOutput(velocityPidOutput);
}
}  // namespace aruwsrc::control::turret
