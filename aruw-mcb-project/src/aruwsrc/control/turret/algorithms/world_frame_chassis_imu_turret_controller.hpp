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

#ifndef WORLD_FRAME_CHASSIS_IMU_TURRET_CONTROLLER_HPP_
#define WORLD_FRAME_CHASSIS_IMU_TURRET_CONTROLLER_HPP_

#include <cstdint>

#include "tap/algorithms/contiguous_float.hpp"
#include "tap/algorithms/smooth_pid.hpp"

#include "aruwsrc/control/turret/turret_subsystem_interface.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control::turret
{
class WorldFrameChassisImuTurretController
{
public:
    /**
     * Updates the `worldFrameYawSetpoint` and runs a world frame position PID controller using the
     * onboard IMU, which is mounted on the chassis.
     *
     * @param[in] drivers Reference to a drivers singleton object.
     * @param[in] dt The time difference between the last time this function was called and the
     *      current time, in milliseconds.
     * @param[in] desiredSetpoint The new desired yaw angle in degrees in the world frame that the
     *      controller will be using.
     * @param[in] chassisFrameInitImuYawAngle The initial yaw IMU angle in degrees in the chassis
     * frame that is defined when the controller is started (in the initialization stage of the
     *      controller).
     * @param[out] worldFrameYawSetpoint World frame yaw setpoint in degrees that is updated each
     *      time the controller is run.
     * @param[out] yawPid The yaw position PID controller to be used to control the turret.
     * @param[out] turretSubsystem The turret subsystem that the controller is controlling. The yaw
     *      setpoint and desired output is updated by the controller.
     */
    static void runYawPidController(
        aruwsrc::Drivers &drivers,
        const uint32_t dt,
        const float desiredSetpoint,
        const float chassisFrameInitImuYawAngle,
        tap::algorithms::ContiguousFloat *worldFrameYawSetpoint,
        tap::algorithms::SmoothPid *yawPid,
        tap::control::turret::TurretSubsystemInterface *turretSubsystem);

    /**
     * Updates the `worldFrameYawSetpoint` and runs a world frame cascade PID controller using the
     * onboard IMU, which is mounted on the chassis.
     *
     * @param[in] drivers Reference to a drivers singleton object.
     * @param[in] dt The time difference between the last time this function was called and the
     *      current time, in milliseconds.
     * @param[in] desiredSetpoint The new desired yaw angle in degrees in the world frame that the
     *      controller will be using.
     * @param[in] chassisFrameInitImuYawAngle The initial yaw IMU angle in the chassis frame in
     *      degrees that is defined when the controller is started (in the initialization stage of
     *      the controller).
     * @param[out] worldFrameYawSetpoint Yaw setpoint in the world frame in degrees that is updated
     *      each time the controller is run.
     * @param[out] yawPositionPid The yaw position PID controller whose output is fed into the
     *      velocity PID controller.
     * @param[out] yawVelocityPid The yaw velocity PID controller that takes input from the position
     *      PID controller and then is used to control the turret.
     * @param[out] turretSubsystem The turret subsystem that the controller is controlling. The yaw
     *      setpoint and desired output is updated by the controller.
     */
    static void runYawCascadePidController(
        aruwsrc::Drivers &drivers,
        const uint32_t dt,
        const float desiredSetpoint,
        const float chassisFrameInitImuYawAngle,
        tap::algorithms::ContiguousFloat *worldFrameYawSetpoint,
        tap::algorithms::SmoothPid *yawPositionPid,
        tap::algorithms::SmoothPid *yawVelocityPid,
        tap::control::turret::TurretSubsystemInterface *turretSubsystem);
};
}  // namespace aruwsrc::control::turret

#endif  // WORLD_FRAME_CHASSIS_IMU_TURRET_CONTROLLER_HPP_
