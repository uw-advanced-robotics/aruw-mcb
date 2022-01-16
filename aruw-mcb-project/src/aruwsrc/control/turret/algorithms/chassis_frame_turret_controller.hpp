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

#ifndef CHASSIS_FRAME_TURRET_CONTROLLER_HPP_
#define CHASSIS_FRAME_TURRET_CONTROLLER_HPP_

#include <cstdint>

#include "tap/algorithms/smooth_pid.hpp"

#include "../turret_subsystem_interface.hpp"

namespace aruwsrc::control::turret
{
class ChassisFrameTurretController
{
public:
    /**
     * Runs a chassis frame position PID controller to control the pitch axis
     * of the turret.
     *
     * @param[in] dt The time difference between the last time this function was called and the
     *      current time, in milliseconds.
     * @param[in] desiredSetpoint The new desired pitch angle in degrees in the chassis frame that
     *      the controller will be using.
     * @param[in] turretCGX See `turret_gravity_compensation.hpp` for details.
     * @param[in] turretCGZ See `turret_gravity_compensation.hpp` for details.
     * @param[in] gravityCompensationMotorOutputMax See `turret_gravity_compensation.hpp` for
     *      details.
     * @param[out] pid The pitch position PID controller to be used to control the pitch axis of the
     *      turret.
     * @param[out] turretSubsystem The turret subsystem that the controller is controlling. The
     *      pitch setpoint and desired output is updated by the controller.
     */
    static void runPitchPidController(
        const uint32_t dt,
        const float desiredSetpoint,
        const float turretCGX,
        const float turretCGZ,
        const float gravityCompensationMotorOutputMax,
        tap::algorithms::SmoothPid *pid,
        aruwsrc::control::turret::TurretSubsystemInterface *turretSubsystem);

    /**
     * Runs a chassis frame cascade PID controller to control the pitch axis
     * of the turret.
     *
     * @param[in] dt The time difference between the last time this function was called and the
     *      current time, in milliseconds.
     * @param[in] desiredSetpoint The new desired pitch angle in degrees in the chassis frame that
     *      the controller will be using.
     * @param[in] turretCGX See `turret_gravity_compensation.hpp` for details.
     * @param[in] turretCGZ See `turret_gravity_compensation.hpp` for details.
     * @param[in] gravityCompensationMotorOutputMax See `turret_gravity_compensation.hpp` for
     *      details.
     * @param[out] positionPid The pitch position PID controller whose input will be a setpoint to
     *      the velocity PID controller.
     * @param[out] velocityPid The pitch velocity PID controller that takes input from the position
     *      PID controller and whose output is used to control the pitch motor.
     * @param[out] turretSubsystem The turret subsystem that the controller is controlling.
     *      The pitch setpoint and desired output is updated by the controller.
     */
    static void runPitchCascadePidController(
        const uint32_t dt,
        const float desiredSetpoint,
        const float turretCGX,
        const float turretCGZ,
        const float gravityCompensationMotorOutputMax,
        tap::algorithms::SmoothPid *positionPid,
        tap::algorithms::SmoothPid *velocityPid,
        aruwsrc::control::turret::TurretSubsystemInterface *turretSubsystem);

    /**
     * Runs a chassis frame position PID controller to control the yaw axis
     * of the turret.
     *
     * @param[in] dt The time difference between the last time this function was called
     *      and the current time, in milliseconds.
     * @param[in] desiredSetpoint The new desired yaw angle in degrees in the chassis frame that the
     *      controller will be using.
     * @param[out] pid The PID controller to be used to control the yaw in the chassis frame.
     * @param[out] turretSubsystem The turret subsystem that the controller is controlling.
     *      The yaw setpoint and desired output is updated by the controller.
     */
    static void runYawPidController(
        const uint32_t dt,
        const float desiredSetpoint,
        tap::algorithms::SmoothPid *pid,
        aruwsrc::control::turret::TurretSubsystemInterface *turretSubsystem);

    /**
     * Runs a chassis frame cascade PID controller to control the yaw axis
     * of the turret.
     *
     * @param[in] dt The time difference between the last time this function was called
     *      and the current time, in milliseconds.
     * @param[in] desiredSetpoint The new desired yaw angle in degrees in the chassis frame that the
     *      controller will be using.
     * @param[out] positionPid The yaw position PID controller whose input will be a setpoint to the
     *      velocity PID controller.
     * @param[out] velocityPid The yaw velocity PID controller that takes input from the position
     * PID controller and whose output is used to control the yaw motor.
     * @param[out] turretSubsystem The turret subsystem that the controller is controlling.
     *      The yaw setpoint and desired output is updated by the controller.
     */
    static void runYawCascadePidController(
        const uint32_t dt,
        const float desiredSetpoint,
        tap::algorithms::SmoothPid *positionPid,
        tap::algorithms::SmoothPid *velocityPid,
        aruwsrc::control::turret::TurretSubsystemInterface *turretSubsystem);
};
}  // namespace aruwsrc::control::turret

#endif  // CHASSIS_FRAME_TURRET_CONTROLLER_HPP_
