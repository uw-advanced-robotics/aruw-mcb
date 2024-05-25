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

#include "tap/drivers.hpp"

namespace aruwsrc::control::turret::algorithms
{
/**
 * Transforms the passed in turret yaw angle in the chassis frame to the world frame (units
 * radians).
 *
 * @param[in] initChassisFrameImuAngle The initial chassis IMU angle, in radians, measured from the
 *      chassis mounted IMU that is captured upon initialization of the chassis IMU world relative
 *      PID controller.
 * @param[in] currChassisFrameImuAngle The current chassis IMU angle, in radians, measured from the
 *      chassis mounted IMU.
 * @param[in] angleToTransform The angle, in radians, to transform. Measured as a turret yaw angle
 *      in the chassis frame.
 * @return A turret yaw angle in radians. `angleToTransform` transformed into the world frame.
 */
static inline WrappedFloat transformChassisFrameYawToWorldFrame(
    const WrappedFloat initChassisFrameImuAngle,
    const WrappedFloat currChassisFrameImuAngle,
    const WrappedFloat angleToTransform)
{
    return angleToTransform + currChassisFrameImuAngle - initChassisFrameImuAngle;
}

/**
 * Transforms the passed in turret yaw angle in the world frame to the chassis frame (units
 * radians).
 *
 * @param[in] initChassisFrameImuAngle The initial chassis IMU angle, in radians, measured from the
 *      chassis mounted IMU that is captured upon initialization of the chassis IMU world relative
 * PID controller.
 * @param[in] currChassisFrameImuAngle The current chassis IMU angle, in radians, measured from the
 *      chassis mounted IMU.
 * @param[in] angleToTransform The angle, in radians to transform. Measured as a turret yaw angle in
 *      the world frame.
 * @return A turret yaw angle in radians. `angleToTransform` transformed into the chassis frame.
 */
static inline WrappedFloat transformWorldFrameYawToChassisFrame(
    const WrappedFloat initChassisFrameImuAngle,
    const WrappedFloat currChassisFrameImuAngle,
    const WrappedFloat angleToTransform)
{
    return angleToTransform - currChassisFrameImuAngle + initChassisFrameImuAngle;
}

/**
 * A helper function for the `run*PidYawWorldFrameController` functions below. Updates the passed in
 * `yawMotor`'s desired chassis frame setpoint and the passed in `worldFrameYawSetpoint`'.
 * Performs necessary limiting of the `worldFrameYawSetpoint` based on the `yawMotor`'s
 * min/max yaw setpoints.
 *
 * @param[in] desiredSetpoint The new user-specified world frame turret yaw angle setpoint, in
 *      radians.
 * @param[in] initChassisFrameImuAngle The initial chassis IMU angle, in radians, measured from the
 *      chassis mounted IMU that is captured upon initialization of the chassis IMU world relative
 *      PID controller.
 * @param[in] currChassisFrameImuAngle The current chassis IMU angle, in radians, measured from the
 *      chassis mounted IMU.
 * @param[out] worldFrameYawSetpoint The limited and wrapped world frame turret yaw setpoint, in
 *      radians. Set to `desiredSetpoint` and then wrapped/limited as necessary.
 * @param[out] yawMotor The turret subsystem whose chassis relative turret yaw angle is
 *      updated by this function.
 */
static inline void updateYawWorldFrameSetpoint(
    const WrappedFloat desiredSetpoint,
    const WrappedFloat chassisFrameInitImuYawAngle,
    const WrappedFloat chassisFrameImuYawAngle,
    WrappedFloat &worldFrameYawSetpoint,
    TurretMotor &yawMotor)
{
    worldFrameYawSetpoint = desiredSetpoint;

    // project target angle in world relative to chassis relative to limit the value
    yawMotor.setChassisFrameSetpoint(transformWorldFrameYawToChassisFrame(
        chassisFrameInitImuYawAngle,
        chassisFrameImuYawAngle,
        worldFrameYawSetpoint));

    if (yawMotor.getConfig().limitMotorAngles)
    {
        // project angle that is limited by the subsystem to world relative again to run the
        // controller. Otherwise use worldFrameYawSetpoint directly.
        worldFrameYawSetpoint = transformChassisFrameYawToWorldFrame(
            chassisFrameInitImuYawAngle,
            chassisFrameImuYawAngle,
            yawMotor.getChassisFrameSetpoint());
    }
}

WorldFrameYawChassisImuTurretController::WorldFrameYawChassisImuTurretController(
    tap::Drivers &drivers,
    TurretMotor &yawMotor,
    const tap::algorithms::SmoothPidConfig &pidConfig)
    : TurretYawControllerInterface(yawMotor),
      drivers(drivers),
      pid(pidConfig),
      worldFrameSetpoint(Angle(0)),
      chassisFrameInitImuYawAngle(Angle(0))
{
}

void WorldFrameYawChassisImuTurretController::initialize()
{
    if (turretMotor.getTurretController() != this)
    {
        pid.reset();

        // revolutions = 0;
        // prevYaw = ;

        chassisFrameInitImuYawAngle = getMpu6500Yaw();
        worldFrameSetpoint = turretMotor.getChassisFrameSetpoint();

        turretMotor.attachTurretController(this);
    }
}

void WorldFrameYawChassisImuTurretController::runController(
    const uint32_t dt,
    const WrappedFloat desiredSetpoint)
{
    // updateRevolutionCounter();

    const WrappedFloat chassisFrameImuYawAngle = getMpu6500Yaw();

    updateYawWorldFrameSetpoint(
        desiredSetpoint,
        chassisFrameInitImuYawAngle,
        chassisFrameImuYawAngle,
        worldFrameSetpoint,
        turretMotor);

    const WrappedFloat worldFrameYawAngle = transformChassisFrameYawToWorldFrame(
        chassisFrameInitImuYawAngle,
        chassisFrameImuYawAngle,
        turretMotor.getChassisFrameMeasuredAngle());

    // position controller based on imu and yaw gimbal angle
    const float positionControllerError =
        turretMotor.getValidMinError(worldFrameSetpoint, worldFrameYawAngle);
    const float pidOutput = pid.runController(
        positionControllerError,
        turretMotor.getChassisFrameVelocity() + modm::toRadian(drivers.mpu6500.getGz()),
        dt);

    turretMotor.setMotorOutput(pidOutput);
}

void WorldFrameYawChassisImuTurretController::setSetpoint(WrappedFloat desiredSetpoint)
{
    updateYawWorldFrameSetpoint(
        desiredSetpoint,
        chassisFrameInitImuYawAngle,
        chassisFrameInitImuYawAngle,
        worldFrameSetpoint,
        turretMotor);
}

WrappedFloat WorldFrameYawChassisImuTurretController::getSetpoint() const
{
    return worldFrameSetpoint;
}

WrappedFloat WorldFrameYawChassisImuTurretController::getMeasurement() const
{
    const WrappedFloat chassisFrameImuYawAngle =
        getMpu6500Yaw();  // NOTE THIS WAS NOT PREVIOUSLY IN RADIANS

    return transformChassisFrameYawToWorldFrame(
        chassisFrameInitImuYawAngle,
        chassisFrameImuYawAngle,
        turretMotor.getChassisFrameMeasuredAngle());
}

bool WorldFrameYawChassisImuTurretController::isOnline() const
{
    return turretMotor.isOnline() && drivers.mpu6500.isRunning();
}

WrappedFloat WorldFrameYawChassisImuTurretController::convertControllerAngleToChassisFrame(
    WrappedFloat controllerFrameAngle) const
{
    const WrappedFloat chassisFrameImuYawAngle = getMpu6500Yaw();

    return transformWorldFrameYawToChassisFrame(
        chassisFrameInitImuYawAngle,
        chassisFrameImuYawAngle,
        controllerFrameAngle);
}

WrappedFloat WorldFrameYawChassisImuTurretController::convertChassisAngleToControllerFrame(
    WrappedFloat chassisFrameAngle) const
{
    const WrappedFloat chassisFrameImuYawAngle = Angle(modm::toRadian(drivers.mpu6500.getYaw()));

    return transformChassisFrameYawToWorldFrame(
        chassisFrameInitImuYawAngle,
        chassisFrameImuYawAngle,
        chassisFrameAngle);
}

}  // namespace aruwsrc::control::turret::algorithms
