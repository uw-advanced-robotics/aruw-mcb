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

#ifndef OTTO_CHASSIS_ORIENTATION_GETTER_HPP_
#define OTTO_CHASSIS_ORIENTATION_GETTER_HPP_

#include "tap/control/odometry/chassis_orientation_getter_interface.hpp"

// Forward declarations
namespace aruwsrc
{
class Drivers;
}
namespace tap::control::turret
{
class TurretSubsystemInterface;
}

namespace aruwsrc::control::odometry
{
/**
 * Otto specific ChassisOrientationGetter
 * 
 * Returns the orientation of the chassis based on subtracting the turret
 * yaw in chassis-frame from the turret IMU's yaw in global frame.
 */
class OttoChassisOrientationGetter
    : public tap::control::odometry::ChassisOrientationGetterInterface
{
public:
    /**
     * @param[in] drivers a pointer to the aruwsrc drivers struct. Used for accessing the
     *      turretMCB IMU
     * @param[in] turret a pointer to the turret used for getting world frame axes. Used to get
     *      yaw angle of chassis relative to turret.
     */
    OttoChassisOrientationGetter(
        aruwsrc::Drivers* drivers,
        tap::control::turret::TurretSubsystemInterface* turret);

    /**
     * Get the current chassis orientation in radians. Will
     * @param[out] output chassis angle in turret-world frame, sweeps from positive x-axis
     *      of field to positive x-axis of chassis. i.e.: rotation around z-axis, positive
     *      z-axis is upwards.
     * @return `true` if valid chassis orientation was available. i.e: true if and only if
     *      turret->isOnline() && drivers->turretMCBCanComm.isConnected()
     */
    bool getChassisOrientation(float* output) final;

private:
    aruwsrc::Drivers* drivers;
    tap::control::turret::TurretSubsystemInterface* turret;
};

}  // namespace aruwsrc::control::odometry

#endif  // OTTO_CHASSIS_ORIENTATION_GETTER_HPP_
