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

#include "world_frame_chassis_imu_turret_controller.hpp"

#include "aruwsrc/drivers.hpp"

namespace aruwsrc::control::turret
{
/**
 * Transforms the passed in turret yaw angle in the chassis frame to the world frame.
 *
 * @param[in] initChassisFrameImuAngle The initial chassis IMU angle, in degrees, measured from the
 *      chassis mounted IMU that is captured upon initialization of the chassis IMU world relative
 *      PID controller.
 * @param[in] currChassisFrameImuAngle The current chassis IMU angle, in degrees, measured from the
 *      chassis mounted IMU.
 * @param[in] angleToTransform The angle, in degrees, to transform. Measured as a turret yaw angle
 *      in the chassis frame.
 * @return A turret yaw angle in degrees. `angleToTransform` transformed into the world frame.
 */
static inline float transformChassisFrameYawToWorldFrame(
    const float initChassisFrameImuAngle,
    const float currChassisFrameImuAngle,
    const float angleToTransform)
{
    return angleToTransform + currChassisFrameImuAngle - initChassisFrameImuAngle;
}

/**
 * Transforms the passed in turret yaw angle in the world frame to the chassis frame.
 *
 * @param[in] initChassisFrameImuAngle The initial chassis IMU angle, in degrees, measured from the
 *      chassis mounted IMU that is captured upon initialization of the chassis IMU world relative
 * PID controller.
 * @param[in] currChassisFrameImuAngle The current chassis IMU angle, in degrees, measured from the
 *      chassis mounted IMU.
 * @param[in] angleToTransform The angle, in degrees to transform. Measured as a turret yaw angle in
 *      the world frame.
 * @return A turret yaw angle in degrees. `angleToTransform` transformed into the chassis frame.
 */
static inline float transformWorldFrameYawToChassisFrame(
    const float initChassisFrameImuAngle,
    const float currChassisFrameImuAngle,
    const float angleToTransform)
{
    return angleToTransform - currChassisFrameImuAngle + initChassisFrameImuAngle;
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
    const float chassisFrameInitImuYawAngle,
    const float chassisFrameImuYawAngle,
    tap::algorithms::ContiguousFloat *worldFrameYawSetpoint,
    TurretSubsystemInterface *turretSubsystem)
{
    worldFrameYawSetpoint->setValue(desiredSetpoint);

    // project target angle in world relative to chassis relative to limit the value
    turretSubsystem->setYawSetpoint(transformWorldFrameYawToChassisFrame(
        chassisFrameInitImuYawAngle,
        chassisFrameImuYawAngle,
        worldFrameYawSetpoint->getValue()));

    if (turretSubsystem->yawLimited())
    {
        // project angle that is limited by the subsystem to world relative again to run the
        // controller. Otherwise use worldFrameYawSetpoint directly.
        worldFrameYawSetpoint->setValue(transformChassisFrameYawToWorldFrame(
            chassisFrameInitImuYawAngle,
            chassisFrameImuYawAngle,
            turretSubsystem->getYawSetpoint()));
    }
}

void WorldFrameChassisImuTurretController::runYawPidController(
    aruwsrc::Drivers &drivers,
    const uint32_t dt,
    const float desiredSetpoint,
    const float chassisFrameInitImuYawAngle,
    tap::algorithms::ContiguousFloat *worldFrameYawSetpoint,
    tap::algorithms::SmoothPid *yawPid,
    TurretSubsystemInterface *turretSubsystem)
{
    const float chassisFrameImuYawAngle = drivers.mpu6500.getYaw();

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
    float positionControllerError = -worldFrameYawSetpoint->difference(worldFrameYawAngle);
    float pidOutput = yawPid->runController(
        positionControllerError,
        turretSubsystem->getYawVelocity() + drivers.mpu6500.getGz(),
        dt);

    turretSubsystem->setYawMotorOutput(pidOutput);
}

void WorldFrameChassisImuTurretController::runYawCascadePidController(
    aruwsrc::Drivers &drivers,
    const uint32_t dt,
    const float desiredSetpoint,
    const float chassisFrameInitImuYawAngle,
    tap::algorithms::ContiguousFloat *worldFrameYawSetpoint,
    tap::algorithms::SmoothPid *yawPositionPid,
    tap::algorithms::SmoothPid *yawVelocityPid,
    TurretSubsystemInterface *turretSubsystem)
{
    const float chassisFrameImuYawAngle = drivers.mpu6500.getYaw();
    const float worldFrameYawVelocity = turretSubsystem->getYawVelocity() + drivers.mpu6500.getGz();

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
    float positionControllerError = -worldFrameYawSetpoint->difference(worldFrameYawAngle);
    float positionPidOut =
        yawPositionPid->runController(positionControllerError, worldFrameYawVelocity, dt);

    float velocityControllerError = positionPidOut - worldFrameYawVelocity;
    float velocityPidOut = yawVelocityPid->runControllerDerivateError(velocityControllerError, dt);

    turretSubsystem->setYawMotorOutput(velocityPidOut);
}
}  // namespace aruwsrc::control::turret
