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

#ifndef TURRET_PID_TURRET_IMU_WORLD_REL_HPP_
#define TURRET_PID_TURRET_IMU_WORLD_REL_HPP_

#include <cstdint>

#include "tap/algorithms/contiguous_float.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/turret/turret_subsystem_interface.hpp"

#include "modm/math/geometry/angle.hpp"

namespace tap
{
class Drivers;
}

namespace aruwsrc::control::turret::turret_imu_world_rel
{
/**
 * Updates the worldFrameYawSetpoint based on the desiredSetpoint and runs the yawPid
 * position PID controller, updating the turretSubsystem's desired setpoint and desired
 * output.
 *
 * @param[in] dt The time difference between the last time this function was called
 *      and the current time.
 * @param[in] desiredSetpoint The new desired yaw angle in the chassis frame that the
 *      controller will be using.
 * @param[in] drivers Pointer to a drivers singleton object.
 * @param[out] worldFrameYawSetpoint The world frame setpoint that is set and limited
 *      to the min/max yaw angeles by this function.
 * @param[in] yawPid The yaw position PID controller to be used to control
 *      the turret.
 * @param[out] turretSubsystem The turret subsystem that the controller is controlling.
 *      The yaw setpoint and desired output is updated by the controller.
 */
void runSinglePidYawWorldFrameController(
    const uint32_t dt,
    const float desiredSetpoint,
    const tap::Drivers *drivers,
    tap::algorithms::ContiguousFloat &worldFrameYawSetpoint,
    tap::algorithms::SmoothPid &yawPid,
    tap::control::turret::TurretSubsystemInterface *turretSubsystem);

/**
 * Updates the worldFrameYawSetpoint based on the desiredSetpoint and runs a cascade
 * PID controller (a position controller that feeds into a velocity controller).
 *
 * @param[in] dt The time difference between the last time this function was called
 *      and the current time.
 * @param[in] desiredSetpoint The new desired yaw angle in the chassis frame that the
 *      controller will be using.
 * @param[in] drivers Pointer to a drivers singleton object.
 * @param[out] worldFrameYawSetpoint The world frame setpoint that is set and limited
 *      to the min/max yaw angeles by this function.
 * @param[in] yawPositionPid The yaw position PID controller whose output is fed into
 *      the velocity PID controller.
 * @param[in] yawVelocityPid The yaw velocity PID controller that takes input from the
 *      position PID controller and then is used to control the turret.
 * @param[out] turretSubsystem The turret subsystem that the controller is controlling.
 *      The yaw setpoint and desired output is updated by the controller.
 */
void runDoublePidYawWorldFrameController(
    const uint32_t dt,
    const float desiredSetpoint,
    const tap::Drivers *drivers,
    tap::algorithms::ContiguousFloat &worldFrameYawSetpoint,
    tap::algorithms::SmoothPid &yawPositionPid,
    tap::algorithms::SmoothPid &yawVelocityPid,
    tap::control::turret::TurretSubsystemInterface *turretSubsystem);

/**
 * Updates the worldFramePitchSetpoint based on the desiredSetpoint and runs a position
 * PID controller, the output of which is fed into the turret subsystem.
 *
 * @param[in] dt The time difference between the last time this function was called
 *      and the current time.
 * @param[in] desiredSetpoint The new desired pitch angle in the chassis frame that the
 *      controller will be using.
 * @param[in] drivers Pointer to a drivers singleton object.
 * @param[in] turretCGX The center of gravity relative to the center of the turret, in the X
 *      (forward/back) direction. Units in millimeters. Positive is forward, negative is backwards.
 * @param[in] turretCGZ The center of gravity relative to the center, in the Z (up/down) direction.
 *      Units in millimeters.
 * @param[in] pitchAngleFromCenter The angle in degrees of the turret pitch, relative to the
 *      horizontal plane.
 * @param[in] gravityCompensationMax The maximum gravity offset to be returned, the gravity is
 *      scaled by this value.
 * @param[out] worldFramePitchSetpoint The world frame setpoint that is set and limited
 *      to the min/max pitch angles by this function.
 * @param[in] pitchPosition The pitch position PID controller whose output is fed into
 *      the velocity PID controller.
 * @param[in] pitchVelocityPid The pitch velocity PID controller that takes input from the
 *      position PID controller and then is used to control the turret.
 * @param[out] turretSubsystem The turret subsystem that the controller is controlling.
 *      The pitch setpoint and desired output is updated by the controller.
 */
void runSinglePidPitchWorldFrameController(
    const uint32_t dt,
    const float desiredSetpoint,
    const tap::Drivers *drivers,
    const float turretCGX,
    const float turretCGZ,
    const float gravityCompensationMax,
    tap::algorithms::ContiguousFloat &worldFramePitchSetpoint,
    tap::algorithms::SmoothPid &pitchPid,
    tap::control::turret::TurretSubsystemInterface *turretSubsystem);

/**
 * Updates the worldFramePitchSetpoint based on the desiredSetpoint and runs a cascade
 * PID controller (a position controller that feeds into a velocity controller), the
 * output of which is fed into the turret subsystem.
 *
 * @param[in] dt The time difference between the last time this function was called
 *      and the current time.
 * @param[in] desiredSetpoint The new desired pitch angle in the chassis frame that the
 *      controller will be using.
 * @param[in] drivers Pointer to a drivers singleton object.
 * @param[in] turretCGX The center of gravity relative to the center of the turret, in the X
 *      (forward/back) direction. Units in millimeters. Positive is forward, negative is backwards.
 * @param[in] turretCGZ The center of gravity relative to the center, in the Z (up/down) direction.
 *      Units in millimeters.
 * @param[in] pitchAngleFromCenter The angle in degrees of the turret pitch, relative to the
 *      horizontal plane.
 * @param[in] gravityCompensationMax The maximum gravity offset to be returned, the gravity is
 *      scaled by this value.
 * @param[out] worldFramePitchSetpoint The world frame setpoint that is set and limited
 *      to the min/max pitch angles by this function.
 * @param[in] pitchPosition The pitch position PID controller whose output is fed into
 *      the velocity PID controller.
 * @param[in] pitchVelocityPid The pitch velocity PID controller that takes input from the
 *      position PID controller and then is used to control the turret.
 * @param[out] turretSubsystem The turret subsystem that the controller is controlling.
 *      The pitch setpoint and desired output is updated by the controller.
 */
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
    tap::control::turret::TurretSubsystemInterface *turretSubsystem);

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
inline float transformChassisFrameToWorldFrame(
    const float turretChassisFrameCurrAngle,
    const float turretWorldFrameCurrAngle,
    const float angleToTransform)
{
    return turretWorldFrameCurrAngle + (angleToTransform - turretChassisFrameCurrAngle);
}

/**
 * Transforms the specified `angleToTransform`, a yaw or pitch angle, from the world frame to the
 * chassis frame.
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
inline float transformWorldFrameValueToChassisFrame(
    const float turretChassisFrameCurrAngle,
    const float turretWorldFrameCurrAngle,
    const float angleToTransform)
{
    return turretChassisFrameCurrAngle + (angleToTransform - turretWorldFrameCurrAngle);
}
}  // namespace aruwsrc::control::turret::turret_imu_world_rel

#endif  //  TURRET_PID_TURRET_IMU_WORLD_REL_HPP_
