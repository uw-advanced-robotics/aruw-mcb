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
#include "tap/architecture/timeout.hpp"

#include "../turret_subsystem.hpp"

#include "turret_controller_interface.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control::turret
{
class TurretSubsystem;
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
 */
class WorldFrameYawChassisImuTurretController final : public TurretYawControllerInterface
{
public:
    /**
     * @param[in] drivers A drivers object that will be queried for IMU information.
     * @param[in] turretSubsystem A `TurretSubsystem` object accessible for children objects to use.
     * @param[in] pidConfig PID configuration struct for the controller.
     */
    WorldFrameYawChassisImuTurretController(
        aruwsrc::Drivers *drivers,
        TurretSubsystem *turretSubsystem,
        const tap::algorithms::SmoothPidConfig &pidConfig);

    void initialize() final;

    /**
     * @see TurretControllerInterface for more details.
     * @param[in] desiredSetpoint The yaw desired setpoint in the world frame.
     */
    void runController(const uint32_t dt, const float desiredSetpoint) final;

    /**
     * @return The yaw setpoint, in the world frame.
     */
    float getSetpoint() const final;

    bool isOnline() const final;

private:
    /** Minimum time during which the PID controller isn't running that will cause the PID
     * controller to be reset when initializing. */
    static constexpr uint32_t RESET_TIME_MS = 100;

    aruwsrc::Drivers *drivers;

    tap::algorithms::SmoothPid pid;

    tap::arch::MilliTimeout resetPidTimeout;

    tap::algorithms::ContiguousFloat worldFrameSetpoint;

    float chassisFrameInitImuYawAngle = 0.0f;
};

}  // namespace aruwsrc::control::turret::algorithms

#endif  // WORLD_FRAME_CHASSIS_IMU_TURRET_CONTROLLER_HPP_
