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

#include "turret_pid_chassis_imu_world_rel.hpp"

#include "tap/drivers.hpp"

namespace aruwsrc::control::turret::chassis_imu_world_rel
{
static inline float transformChassisFrameYawToWorldFrame(
    const float initChassisFrameImuAngle,
    const float currChassisFrameImuAngle,
    const float angleToTransform)
{
    return angleToTransform + currChassisFrameImuAngle - initChassisFrameImuAngle;
}

static inline float transformWorldFrameYawToChassisFrame(
    const float initChassisFrameImuAngle,
    const float currChassisFrameImuAngle,
    const float angleToTransform)
{
    return angleToTransform - currChassisFrameImuAngle + initChassisFrameImuAngle;
}

static inline void updateYawWorldFrameSetpoint(
    const float desiredSetpoint,
    const float chassisFrameInitImuYawAngle,
    const float chassisFrameImuYawAngle,
    tap::algorithms::ContiguousFloat &worldFrameYawSetpoint,
    tap::control::turret::TurretSubsystemInterface *turretSubsystem)
{
    worldFrameYawSetpoint.setValue(desiredSetpoint);

    // project target angle in world relative to chassis relative to limit the value
    turretSubsystem->setYawSetpoint(transformWorldFrameYawToChassisFrame(
        chassisFrameInitImuYawAngle,
        chassisFrameImuYawAngle,
        worldFrameYawSetpoint.getValue()));

    if (turretSubsystem->yawLimited())
    {
        // project angle that is limited by the subsystem to world relative again to run the
        // controller. Otherwise use worldFrameYawSetpoint directly.
        worldFrameYawSetpoint.setValue(transformChassisFrameYawToWorldFrame(
            chassisFrameInitImuYawAngle,
            chassisFrameImuYawAngle,
            turretSubsystem->getYawSetpoint()));
    }
}

void runSinglePidYawWorldFrameController(
    const uint32_t dt,
    const float desiredSetpoint,
    const float chassisFrameInitImuYawAngle,
    tap::algorithms::ContiguousFloat &worldFrameYawSetpoint,
    tap::Drivers *drivers,
    tap::control::turret::TurretSubsystemInterface *turretSubsystem,
    tap::algorithms::SmoothPid &yawPid)
{
    const float chassisFrameImuYawAngle = drivers->mpu6500.getYaw();

    updateYawWorldFrameSetpoint(
        desiredSetpoint,
        chassisFrameInitImuYawAngle,
        chassisFrameImuYawAngle,
        worldFrameYawSetpoint,
        turretSubsystem);

    float worldFrameYawAngle = transformChassisFrameYawToWorldFrame(
        chassisFrameInitImuYawAngle,
        chassisFrameImuYawAngle,
        turretSubsystem->getCurrentYawValue().getValue());

    // position controller based on imu and yaw gimbal angle
    float positionControllerError = -worldFrameYawSetpoint.difference(worldFrameYawAngle);
    float pidOutput = yawPid.runController(
        positionControllerError,
        turretSubsystem->getYawVelocity() + drivers->mpu6500.getGz(),
        dt);

    turretSubsystem->setYawMotorOutput(pidOutput);
}

void runDoublePidYawWorldFrameController(
    const uint32_t dt,
    const float desiredSetpoint,
    const float chassisFrameInitImuYawAngle,
    tap::algorithms::ContiguousFloat &worldFrameYawSetpoint,
    tap::Drivers *drivers,
    tap::control::turret::TurretSubsystemInterface *turretSubsystem,
    tap::algorithms::SmoothPid &yawPositionPid,
    tap::algorithms::SmoothPid &yawVelocityPid)
{
    const float chassisFrameImuYawAngle = drivers->mpu6500.getYaw();
    const float worldFrameYawVelocity =
        turretSubsystem->getYawVelocity() + drivers->mpu6500.getGz();

    updateYawWorldFrameSetpoint(
        desiredSetpoint,
        chassisFrameInitImuYawAngle,
        chassisFrameImuYawAngle,
        worldFrameYawSetpoint,
        turretSubsystem);

    float worldFrameYawAngle = transformChassisFrameYawToWorldFrame(
        chassisFrameInitImuYawAngle,
        chassisFrameImuYawAngle,
        turretSubsystem->getCurrentYawValue().getValue());

    // position controller based on imu and yaw gimbal angle
    float positionControllerError = -worldFrameYawSetpoint.difference(worldFrameYawAngle);
    float positionPidOut =
        yawPositionPid.runController(positionControllerError, worldFrameYawVelocity, dt);

    float velocityControllerError = positionPidOut - worldFrameYawVelocity;
    float velocityPidOut = yawVelocityPid.runControllerDerivateError(velocityControllerError, dt);

    turretSubsystem->setYawMotorOutput(velocityPidOut);
}
}  // namespace aruwsrc::control::turret::chassis_imu_world_rel
