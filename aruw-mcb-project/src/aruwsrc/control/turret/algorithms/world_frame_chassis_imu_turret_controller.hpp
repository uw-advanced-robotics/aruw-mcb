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
    void runController(const uint32_t dt, const float desiredSetpoint) final;

    /// @return World frame yaw angle setpoint, refer to top level documentation for more details.
    void setSetpoint(float desiredSetpoint) final;

    /// @return world frame yaw angle measurement, refer to top level documentation for more
    /// details.
    float getMeasurement() const final;

    /**
     * @return The yaw setpoint, in the world frame.
     */
    float getSetpoint() const final;

    bool isOnline() const final;

    float convertControllerAngleToChassisFrame(float controllerFrameAngle) const final;

    float convertChassisAngleToControllerFrame(float chassisFrameAngle) const final;

private:
    tap::Drivers &drivers;

    tap::algorithms::SmoothPid pid;

    int revolutions = 0;
    float prevYaw = 0;

    float worldFrameSetpoint = 0;

    float chassisFrameInitImuYawAngle = 0.0f;

    float getMpu6500YawUnwrapped() const
    {
        return drivers.mpu6500.getYawRadians() + M_TWOPI * revolutions;
    }

    /**
     * Updates the mpu6500 yaw revolution counter and the prevYaw value (which is used to update the
     * revolution counter). This is the only function that should be used to update `prevYaw` or
     * `revolutions`.
     */
    void updateRevolutionCounter()
    {
        const float newYaw = drivers.mpu6500.getYawRadians();
        const float diff = newYaw - prevYaw;
        prevYaw = newYaw;
        if (diff < -M_PI)
        {
            revolutions++;
        }
        else if (diff > M_PI)
        {
            revolutions--;
        }
    }
};

}  // namespace aruwsrc::control::turret::algorithms

#endif  // WORLD_FRAME_CHASSIS_IMU_TURRET_CONTROLLER_HPP_
