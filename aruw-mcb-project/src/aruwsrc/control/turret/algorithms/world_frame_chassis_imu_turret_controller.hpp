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

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/algorithms/wrapped_float.hpp"
#include "tap/drivers.hpp"

#include "../turret_subsystem.hpp"

#include "turret_controller_interface.hpp"

using namespace tap::algorithms;

namespace aruwsrc::control::turret
{
class TurretMotor;
}

namespace aruwsrc::control::turret::algorithms
{
/**
 * World frame turret yaw controller. Requires that the development board type A is mounted rigidly
 * to the chassis and is properly initialized. Runs a single position PID controller to control the
 * turret yaw. Feedback from the mpu6500 and the turret yaw encoder used to determine the world
 * frame turret angle.
 *
 * Implements TurretControllerInterface interface, see parent class comment for details.
 *
 * @note Upon initialization of the controller, the world frame zero point is set to the current IMU
 * yaw angle.
 */
class WorldFrameYawChassisImuTurretController final : public TurretYawControllerInterface
{
public:
    /**
     * @param[in] drivers A drivers object that will be queried for IMU information.
     * @param[in] yawMotor A `TurretMotor` object accessible for children objects to use.
     * @param[in] pidConfig PID configuration struct for the controller.
     */
    WorldFrameYawChassisImuTurretController(
        tap::Drivers &drivers,
        TurretMotor &yawMotor,
        const tap::algorithms::SmoothPidConfig &pidConfig);

    void initialize() final;

    /**
     * @see TurretControllerInterface for more details.
     * @param[in] desiredSetpoint The yaw desired setpoint in the world frame.
     */
    void runController(const uint32_t dt, const WrappedFloat desiredSetpoint) final;

    /// @return World frame yaw angle setpoint, refer to top level documentation for more details.
    void setSetpoint(WrappedFloat desiredSetpoint) final;

    /// @return world frame yaw angle measurement, refer to top level documentation for more
    /// details.
    WrappedFloat getMeasurement() const final;

    /**
     * @return The yaw setpoint, in the world frame.
     */
    WrappedFloat getSetpoint() const final;

    bool isOnline() const final;

    WrappedFloat convertControllerAngleToChassisFrame(
        WrappedFloat controllerFrameAngle) const final;

    WrappedFloat convertChassisAngleToControllerFrame(WrappedFloat chassisFrameAngle) const final;

private:
    tap::Drivers &drivers;

    tap::algorithms::SmoothPid pid;

    WrappedFloat worldFrameSetpoint;

    WrappedFloat chassisFrameInitImuYawAngle;

    inline WrappedFloat getMpu6500Yaw() const
    {
        return Angle::fromDegrees(drivers.mpu6500.getYaw());
    }
};

}  // namespace aruwsrc::control::turret::algorithms

#endif  // WORLD_FRAME_CHASSIS_IMU_TURRET_CONTROLLER_HPP_
