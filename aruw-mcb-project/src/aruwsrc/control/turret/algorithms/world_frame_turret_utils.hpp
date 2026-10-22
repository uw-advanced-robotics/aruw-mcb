/*
 * Copyright (c) 2020-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef WORLD_FRAME_TURRET_UTILS_HPP_
#define WORLD_FRAME_TURRET_UTILS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/algorithms/transforms/transform.hpp"
#include "tap/algorithms/wrapped_float.hpp"

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
static inline WrappedFloat transformChassisFrameToWorldFrame(
    const WrappedFloat turretChassisFrameCurrAngle,
    const WrappedFloat turretWorldFrameCurrAngle,
    const WrappedFloat angleToTransform)
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
static inline WrappedFloat transformWorldFrameValueToChassisFrame(
    const WrappedFloat turretChassisFrameCurrAngle,
    const WrappedFloat turretWorldFrameCurrAngle,
    const WrappedFloat angleToTransform)
{
    return turretChassisFrameCurrAngle + (angleToTransform - turretWorldFrameCurrAngle);
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
static inline void updateWorldFrameSetpoint(
    const WrappedFloat desiredSetpoint,
    const WrappedFloat chassisFrameMeasurement,
    const WrappedFloat worldFrameMeasurement,
    WrappedFloat &worldFrameSetpoint,
    TurretMotor &turretMotor)
{
    worldFrameSetpoint = desiredSetpoint;

    // transform target angle from turret imu relative to chassis relative
    // to keep turret/command setpoints synchronized

    turretMotor.setChassisFrameSetpoint(transformWorldFrameValueToChassisFrame(
        chassisFrameMeasurement,
        worldFrameMeasurement,
        worldFrameSetpoint));

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
}  // namespace aruwsrc::control::turret::algorithms

#endif  // WORLD_FRAME_TURRET_UTILS_HPP_