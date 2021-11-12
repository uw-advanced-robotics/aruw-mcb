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

#include "turret_pid_turret_imu_world_rel.hpp"

#include "tap/drivers.hpp"

#include "turret_gravity_compensation.hpp"

namespace aruwsrc::control::turret::turret_imu_world_rel
{
static inline void updateYawWorldFrameSetpoint(
    const float desiredSetpoint,
    const float chassisFrameYaw,
    const float worldFrameYawAngle,
    tap::algorithms::ContiguousFloat &worldFrameYawSetpoint,
    tap::control::turret::TurretSubsystemInterface *turretSubsystem)
{
    worldFrameYawSetpoint.setValue(desiredSetpoint);

    // transform target angle from turret imu relative to chassis relative
    // to keep turret/command setpoints synchronized
    turretSubsystem->setYawSetpoint(transformWorldFrameValueToChassisFrame(
        chassisFrameYaw,
        worldFrameYawAngle,
        worldFrameYawSetpoint.getValue()));

    if (turretSubsystem->yawLimited())
    {
        // transform angle that is limited by subsystem to world relative again to run the
        // controller
        worldFrameYawSetpoint.setValue(transformChassisFrameToWorldFrame(
            chassisFrameYaw,
            worldFrameYawAngle,
            turretSubsystem->getYawSetpoint()));
    }
}

void runSinglePidYawWorldFrameController(
    const uint32_t dt,
    const float desiredSetpoint,
    const tap::Drivers *drivers,
    tap::algorithms::ContiguousFloat &worldFrameYawSetpoint,
    tap::algorithms::SmoothPid &yawPid,
    tap::control::turret::TurretSubsystemInterface *turretSubsystem)
{
    const float chassisFrameYaw = turretSubsystem->getCurrentYawValue().getValue();
    const float worldFrameYawAngle = drivers->turretMCBCanComm.getYaw();
    const float worldFrameYawVelocity = drivers->turretMCBCanComm.getGz();

    updateYawWorldFrameSetpoint(
        desiredSetpoint,
        chassisFrameYaw,
        worldFrameYawAngle,
        worldFrameYawSetpoint,
        turretSubsystem);

    // position controller based on imu and yaw gimbal angle,
    // precisly, - (yawActual - worldFrameYawSetpoint), or more obvious,
    // worldFrameYawSetpoint - yawActual
    float positionControllerError = -worldFrameYawSetpoint.difference(worldFrameYawAngle);
    float pidOutput = yawPid.runController(positionControllerError, worldFrameYawVelocity, dt);

    turretSubsystem->setYawMotorOutput(pidOutput);
}

void runDoublePidYawWorldFrameController(
    const uint32_t dt,
    const float desiredSetpoint,
    const tap::Drivers *drivers,
    tap::algorithms::ContiguousFloat &worldFrameYawSetpoint,
    tap::algorithms::SmoothPid &yawPositionPid,
    tap::algorithms::SmoothPid &yawVelocityPid,
    tap::control::turret::TurretSubsystemInterface *turretSubsystem)
{
    const float chassisFrameYaw = turretSubsystem->getCurrentYawValue().getValue();
    const float worldFrameYawAngle = drivers->turretMCBCanComm.getYaw();
    const float worldFrameYawVelocity = drivers->turretMCBCanComm.getGz();

    updateYawWorldFrameSetpoint(
        desiredSetpoint,
        chassisFrameYaw,
        worldFrameYawAngle,
        worldFrameYawSetpoint,
        turretSubsystem);

    // position controller based on imu and yaw gimbal angle,
    // precisly, - (yawActual - worldFrameYawSetpoint), or more obvious,
    // worldFrameYawSetpoint - yawActual
    float positionControllerError = -worldFrameYawSetpoint.difference(worldFrameYawAngle);
    float positionPidOutput =
        yawPositionPid.runController(positionControllerError, worldFrameYawVelocity, dt);

    float velocityControllerError = positionPidOutput - worldFrameYawVelocity;
    float velocityPidOutput =
        yawVelocityPid.runControllerDerivateError(velocityControllerError, dt);

    turretSubsystem->setYawMotorOutput(velocityPidOutput);
}

static inline void updatePitchWorldFrameSetpoint(
    const float desiredSetpoint,
    const float worldFramePitchAngle,
    tap::algorithms::ContiguousFloat &worldFramePitchSetpoint,
    tap::control::turret::TurretSubsystemInterface *turretSubsystem)
{
    worldFramePitchSetpoint.setValue(desiredSetpoint);

    // Project user desired setpoint that is in world relative to chassis relative
    // to limit the value
    turretSubsystem->setPitchSetpoint(transformWorldFrameValueToChassisFrame(
        turretSubsystem->getCurrentPitchValue().getValue(),
        worldFramePitchAngle,
        worldFramePitchSetpoint.getValue()));

    // Project angle limited by the tap::control::turret::TurretSubsystemInterface back to world
    // relative to use the value
    worldFramePitchSetpoint.setValue(transformChassisFrameToWorldFrame(
        turretSubsystem->getCurrentPitchValue().getValue(),
        worldFramePitchAngle,
        turretSubsystem->getPitchSetpoint()));
}

void runSinglePidPitchWorldFrameController(
    const uint32_t dt,
    const float desiredSetpoint,
    const tap::Drivers *drivers,
    const float turretCGX,
    const float turretCGZ,
    const float gravityCompensationMax,
    tap::algorithms::ContiguousFloat &worldFramePitchSetpoint,
    tap::algorithms::SmoothPid &pitchPid,
    tap::control::turret::TurretSubsystemInterface *turretSubsystem)
{
    const float turretWorldFramePitchAngle = drivers->turretMCBCanComm.getPitch();
    const float turretWorldFramePitchVelocity = drivers->turretMCBCanComm.getGx();

    updatePitchWorldFrameSetpoint(
        desiredSetpoint,
        turretWorldFramePitchAngle,
        worldFramePitchSetpoint,
        turretSubsystem);

    // Compute error between pitch target and reference angle
    float positionControllerError = -worldFramePitchSetpoint.difference(turretWorldFramePitchAngle);

    float pidOutput =
        pitchPid.runController(positionControllerError, turretWorldFramePitchVelocity, dt);

    pidOutput += computeGravitationalForceOffset(
        turretCGX,
        turretCGZ,
        turretSubsystem->getPitchAngleFromCenter(),
        gravityCompensationMax);

    turretSubsystem->setPitchMotorOutput(pidOutput);
}

void runDoublePidPitchWorldFrameController(
    const uint32_t dt,
    const float desiredSetpoint,
    const tap::Drivers *drivers,
    const float turretCGX,
    const float turretCGZ,
    const float gravityCompensationMax,
    tap::algorithms::ContiguousFloat &worldFramePitchSetpoint,
    tap::algorithms::SmoothPid &pitchPositionPid,
    tap::algorithms::SmoothPid &pitchVelocityPid,
    tap::control::turret::TurretSubsystemInterface *turretSubsystem)
{
    const float worldFramePitchAngle = drivers->turretMCBCanComm.getPitch();
    const float worldFramePitchVelocity = drivers->turretMCBCanComm.getGx();

    updatePitchWorldFrameSetpoint(
        desiredSetpoint,
        worldFramePitchAngle,
        worldFramePitchSetpoint,
        turretSubsystem);

    // Compute error between pitch target and reference angle
    float positionControllerError = -worldFramePitchSetpoint.difference(worldFramePitchAngle);
    float positionPidOutput =
        pitchPositionPid.runController(positionControllerError, worldFramePitchVelocity, dt);

    float velocityControllerError = positionPidOutput - worldFramePitchVelocity;
    float velocityPidOutput =
        pitchVelocityPid.runControllerDerivateError(velocityControllerError, dt);

    velocityPidOutput += computeGravitationalForceOffset(
        turretCGX,
        turretCGZ,
        turretSubsystem->getPitchAngleFromCenter(),
        gravityCompensationMax);

    turretSubsystem->setPitchMotorOutput(velocityPidOutput);
}
}  // namespace aruwsrc::control::turret::turret_imu_world_rel
