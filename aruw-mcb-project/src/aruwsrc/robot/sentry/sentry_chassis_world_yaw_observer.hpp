/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SENTRY_CHASSIS_WORLD_YAW_OBSERVER_HPP_
#define SENTRY_CHASSIS_WORLD_YAW_OBSERVER_HPP_

#include "tap/algorithms/odometry/chassis_world_yaw_observer_interface.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"

#include "aruwsrc/control/turret/yaw_turret_subsystem.hpp"

namespace aruwsrc::sentry
{
/**
 * @brief Sentry specific ChassisWorldYawObserverInterface implementation
 *
 * Returns the orientation of the chassis based on the orientation of the turret-major IMU
 *
 * @see tap::algorithms::odometry::ChassisWorldYawObserverInterface
 */
class SentryChassisWorldYawObserver
    : public tap::algorithms::odometry::ChassisWorldYawObserverInterface
{
public:
    /**
     * @param[in] drivers a pointer to the tap drivers struct. Used for accessing the
     * turretMCB IMU
     * @param[in] turretSubsystem a reference to the turret used for getting world frame axes. Used
     * to get yaw angle of chassis relative to turret. This must be the same turret that the IMU on
     * CAN bus 1 is attached to.
     */
    SentryChassisWorldYawObserver(
        tap::communication::sensors::imu::ImuInterface &imu,
        aruwsrc::control::turret::YawTurretSubsystem &turretMajor);

    /**
     * Get the current chassis yaw in radians.
     *
     * @param[out] yaw chassis yaw in turret-world frame, sweeps from positive x-axis
     *      of field to positive x-axis of chassis. i.e.: rotation around z-axis, positive
     *      z-axis is upwards.
     *      Normalized to the range (-pi, pi).
     *
     * @return `true` if valid chassis orientation was available. i.e: true if and only if
     *      turret->isOnline() && turretSubsystem.getChassisMCB()->isConnected()
     */
    bool getChassisWorldYaw(float *yaw) const final;

    void overrideChassisYaw(float newYaw);

private:
    tap::communication::sensors::imu::ImuInterface &imu;
    aruwsrc::control::turret::YawTurretSubsystem &turretMajor;

    // error factor since we don't know how to reset the imu to some non-zero value outright
    float offset = 0.0f;
};

}  // namespace aruwsrc::sentry

#endif  // SENTRY_CHASSIS_WORLD_YAW_OBSERVER_HPP_
