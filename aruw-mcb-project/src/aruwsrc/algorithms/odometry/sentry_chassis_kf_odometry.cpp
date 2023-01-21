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

#include "sentry_chassis_kf_odometry.hpp"

#include "tap/drivers.hpp"

namespace aruwsrc::algorithms::odometry
{
SentryChassisKFOdometry::SentryChassisKFOdometry(
    tap::Drivers& drivers,
    const aruwsrc::control::sentry::drive::SentryDriveSubsystem& chassis,
    const aruwsrc::control::turret::TurretSubsystem& turret)
    : chassis(chassis),
      chassisYawObserver(&drivers, turret),
      chassisIMU(drivers.mpu6500),
      kf(KF_A, KF_C, KF_Q, KF_R, KF_P0)
{
    float initialX[static_cast<int>(OdomState::NUM_STATES)] = {chassis.getAbsolutePosition(), 0, 0};
    kf.init(initialX);
}

void SentryChassisKFOdometry::update()
{
    if (!chassisYawObserver.getChassisWorldYaw(&chassisYaw))
    {
        chassisYaw = 0;
    }

    // Assume 0 velocity/acceleration in z direction.
    float y[static_cast<int>(OdomInput::NUM_INPUTS)] = {};

    // Absolute position is given in mm, so we should convert back to meters.
    y[static_cast<int>(OdomInput::POS_Y)] = chassis.getAbsolutePosition() * 1E-3;
    y[static_cast<int>(OdomInput::VEL_Y)] = chassis.getActualVelocityChassisRelative()[1][0];
    y[static_cast<int>(OdomInput::ACC_Y)] = chassisIMU.getAy();

    // Update the Kalman filter. A new state estimate is available after this call.
    kf.performUpdate(y);

    // Update the location and velocity accessor objects with values from the state vector.
    updateChassisStateFromKF();
}

void SentryChassisKFOdometry::updateChassisStateFromKF()
{
    const auto& x = kf.getStateVectorAsMatrix();

    // Update odometry velocity.
    velocity.x = 0;
    velocity.y = x[static_cast<int>(OdomState::VEL_Y)];

    // Update chassis orientation.
    // For this implementation, we assume that the sentry chassis does not yaw on the rail.
    location.setOrientation(0);
    location.setPosition(0, x[static_cast<int>(OdomState::POS_Y)]);
}

}  // namespace aruwsrc::algorithms::odometry
