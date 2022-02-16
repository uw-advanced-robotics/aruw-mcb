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

namespace aruwsrc::control::turret::algorithms
{
/**
 * Transforms the passed in turret yaw angle in the chassis frame to the world frame (units
 * degrees).
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
 * Transforms the passed in turret yaw angle in the world frame to the chassis frame (units
 * degrees).
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
    tap::control::turret::TurretSubsystemInterface *turretSubsystem)
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

WorldFrameYawChassisImuTurretController::WorldFrameYawChassisImuTurretController(
    aruwsrc::Drivers *drivers,
    TurretSubsystem *turretSubsystem,
    const tap::algorithms::SmoothPidConfig &pidConfig)
    : TurretYawControllerInterface(turretSubsystem),
      drivers(drivers),
      pid(pidConfig),
      worldFrameSetpoint(0, 0, 360)
{
}

void WorldFrameYawChassisImuTurretController::initialize()
{
    if (resetPidTimeout.isExpired())
    {
        pid.reset();
    }
    chassisFrameInitImuYawAngle = drivers->mpu6500.getYaw();
    worldFrameSetpoint.setValue(turretSubsystem->getYawSetpoint());
}

void WorldFrameYawChassisImuTurretController::runController(
    const uint32_t dt,
    const float desiredSetpoint)
{
    const float chassisFrameImuYawAngle = drivers->mpu6500.getYaw();

    updateYawWorldFrameSetpoint(
        desiredSetpoint,
        chassisFrameInitImuYawAngle,
        chassisFrameImuYawAngle,
        &worldFrameSetpoint,
        turretSubsystem);

    float worldFrameYawAngle = transformChassisFrameYawToWorldFrame(
        chassisFrameInitImuYawAngle,
        chassisFrameImuYawAngle,
        turretSubsystem->getCurrentYawValue().getValue());

    // position controller based on imu and yaw gimbal angle
    float positionControllerError = -worldFrameSetpoint.difference(worldFrameYawAngle);
    float pidOutput = pid.runController(
        positionControllerError,
        turretSubsystem->getYawVelocity() + drivers->mpu6500.getGz(),
        dt);

    turretSubsystem->setYawMotorOutput(pidOutput);

    resetPidTimeout.restart(RESET_TIME_MS);
}

float WorldFrameYawChassisImuTurretController::getSetpoint() const
{
    return worldFrameSetpoint.getValue();
}

bool WorldFrameYawChassisImuTurretController::isOnline() const
{
    return turretSubsystem->isOnline() && drivers->mpu6500.isRunning();
}

}  // namespace aruwsrc::control::turret::algorithms
