/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "aruwsrc/control/turret/yaw_turret_subsystem.hpp"

namespace aruwsrc::sentry
{
/**
 * @brief Sentry specific ChassisWorldYawObserverInterface implementation
 *
 * Returns the orientation of the chassis based on subtracting the turret
 * yaw in chassis-frame from the turret IMU's yaw in global frame.
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
        const aruwsrc::control::turret::YawTurretSubsystem& turretSubsystem,
        const aruwsrc::control::turret::TurretSubsystem& turretMinorLeftSubsystem,
        const aruwsrc::control::turret::TurretSubsystem& turretMinorRightSubsystem);  // Could do sensor fusion in the future but for now the malewife turret is unused

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
    bool getChassisWorldYaw(float* yaw) const final;


    void overrideChassisYaw(float newYaw);

    mutable float lastGottenYaw;
    mutable float turretWorldYawRadians;
    mutable float turretMinorMajorYawRadians;
    mutable float turretMajorChassisYawRadians;
private:
    const aruwsrc::control::turret::YawTurretSubsystem& turretMajorSubsystem;
    //Girlboss
    const aruwsrc::control::turret::TurretSubsystem& turretMinorLeftSubsystem;
    //Malewife
    const aruwsrc::control::turret::TurretSubsystem& turretMinorRightSubsystem;

    // error factor since we don't know how to reset the imu to some non-zero value outright
    float offset = 0.0f;
};

}  // namespace aruwsrc::sentry

#endif  // SENTRY_CHASSIS_WORLD_YAW_OBSERVER_HPP_
