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

#ifndef TURRET_PID_CHASSIS_REL_HPP_
#define TURRET_PID_CHASSIS_REL_HPP_

#include <cstdint>

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/turret/turret_subsystem_interface.hpp"

namespace aruwsrc::control::turret::chassis_rel
{
/**
 * Runs a chassis frame position PID controller to control the pitch axis
 * of the turret.
 *
 * @param[in] dt The time difference between the last time this function was called
 *      and the current time.
 * @param[in] desiredSetpoint The new desired pitch angle in the chassis frame that the
 *      controller will be using.
 * @param[in] turretCGX The center of gravity relative to the center of the turret, in the X
 *      (forward/back) direction. Units in millimeters. Positive is forward, negative is backwards.
 * @param[in] turretCGZ The center of gravity relative to the center, in the Z (up/down) direction.
 *      Units in millimeters.
 * @param[in] pitchAngleFromCenter The angle in degrees of the turret pitch, relative to the
 *      horizontal plane.
 * @param[in] gravityCompensationMax The maximum gravity offset to be returned, the gravity is
 *      scaled by this value.
 * @param[in] pid The pitch position PID controller to be used to control
 *      the turret.
 * @param[out] turretSubsystem The turret subsystem that the controller is controlling.
 *      The pitch setpoint and desired output is updated by the controller.
 */
void runSinglePidPitchChassisFrameController(
    const uint32_t dt,
    const float desiredSetpoint,
    const float turretCGX,
    const float turretCGZ,
    const float gravityCompensationMax,
    tap::algorithms::SmoothPid &pid,
    tap::control::turret::TurretSubsystemInterface *turretSubsystem);

/**
 * Runs a chassis frame cascade PID controller to control the pitch axis
 * of the turret.
 *
 * @param[in] dt The time difference between the last time this function was called
 *      and the current time.
 * @param[in] desiredSetpoint The new desired pitch angle in the chassis frame that the
 *      controller will be using.
 * @param[in] turretCGX The center of gravity relative to the center of the turret, in the X
 *      (forward/back) direction. Units in millimeters. Positive is forward, negative is backwards.
 * @param[in] turretCGZ The center of gravity relative to the center, in the Z (up/down) direction.
 *      Units in millimeters.
 * @param[in] pitchAngleFromCenter The angle in degrees of the turret pitch, relative to the
 *      horizontal plane.
 * @param[in] gravityCompensationMax The maximum gravity offset to be returned, the gravity is
 *      scaled by this value.
 * @param[in] positionPid The pitch position PID controller whose input will be a setpoint to the
 * velocity PID controller.
 * @param[in] velocityPid The pitch velocity PID controller that takes input from the position PID
 *      controller and whose output is used to control the pitch motor.
 * @param[out] turretSubsystem The turret subsystem that the controller is controlling.
 *      The pitch setpoint and desired output is updated by the controller.
 */
void runDoublePidPitchChassisFrameController(
    const uint32_t dt,
    const float desiredSetpoint,
    const float turretCGX,
    const float turretCGZ,
    const float gravityCompensationMax,
    tap::algorithms::SmoothPid &positionPid,
    tap::algorithms::SmoothPid &velocityPid,
    tap::control::turret::TurretSubsystemInterface *turretSubsystem);

/**
 * Runs a chassis frame position PID controller to control the yaw axis
 * of the turret.
 *
 * @param[in] dt The time difference between the last time this function was called
 *      and the current time.
 * @param[in] desiredSetpoint The new desired yaw angle in the chassis frame that the
 *      controller will be using.
 * @param[in] pid The PID controller to be used to control the yaw in the chassis frame.
 * @param[out] turretSubsystem The turret subsystem that the controller is controlling.
 *      The yaw setpoint and desired output is updated by the controller.
 */
void runSinglePidYawChassisFrameController(
    const uint32_t dt,
    const float desiredSetpoint,
    tap::algorithms::SmoothPid &pid,
    tap::control::turret::TurretSubsystemInterface *turretSubsystem);

/**
 * Runs a chassis frame cascade PID controller to control the yaw axis
 * of the turret.
 *
 * @param[in] dt The time difference between the last time this function was called
 *      and the current time.
 * @param[in] desiredSetpoint The new desired yaw angle in the chassis frame that the
 *      controller will be using.
 * @param[in] positionPid The yaw position PID controller whose input will be a setpoint to the
 *      velocity PID controller.
 * @param[in] velocityPid The yaw velocity PID controller that takes input from the position PID
 *      controller and whose output is used to control the yaw motor.
 * @param[out] turretSubsystem The turret subsystem that the controller is controlling.
 *      The yaw setpoint and desired output is updated by the controller.
 */
void runDoublePidYawChassisFrameController(
    const uint32_t dt,
    const float desiredSetpoint,
    tap::algorithms::SmoothPid &positionPid,
    tap::algorithms::SmoothPid &velocityPid,
    tap::control::turret::TurretSubsystemInterface *turretSubsystem);
}  // namespace aruwsrc::control::turret::chassis_rel

#endif  // TURRET_PID_CHASSIS_REL_HPP_
