/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef ENGINEER_CHASSIS_WORLD_YAW_OBSERVER_HPP_
#define ENGINEER_CHASSIS_WORLD_YAW_OBSERVER_HPP_

#include "tap/algorithms/odometry/chassis_world_yaw_observer_interface.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"

namespace aruwsrc::engineer::algorithms::odometry
{
/**
 * @brief Otto specific ChassisWorldYawObserverInterface implementation
 *
 * Returns the orientation of the chassis based on subtracting the turret
 * yaw in chassis-frame from the turret IMU's yaw in global frame.
 *
 * @see tap::algorithms::odometry::ChassisWorldYawObserverInterface
 */
class EngineerChassisWorldYawObserver
    : public tap::algorithms::odometry::ChassisWorldYawObserverInterface
{
public:
    EngineerChassisWorldYawObserver(
        const tap::communication::sensors::imu::ImuInterface& chassisImu)
        : chassisImu(chassisImu)
    {
    }

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
    bool getChassisWorldYaw(float* yaw) const final
    {
        *yaw = chassisImu.getYaw();
        return true;
    }

private:
    const tap::communication::sensors::imu::ImuInterface& chassisImu;
};

}  // namespace aruwsrc::engineer::algorithms::odometry

#endif  // ENGINEER_CHASSIS_WORLD_YAW_OBSERVER_HPP_
