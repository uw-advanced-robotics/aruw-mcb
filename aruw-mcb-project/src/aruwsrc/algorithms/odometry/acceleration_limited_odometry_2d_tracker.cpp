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

#include "acceleration_limited_odometry_2d_tracker.hpp"

#include <cassert>

#include "tap/algorithms/odometry/chassis_world_yaw_observer_interface.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"

namespace aruwsrc::algorithms::odometry
{
AccelerationLimitedOdometry2DTracker::AccelerationLimitedOdometry2DTracker(
    const tap::control::chassis::ChassisSubsystemInterface& chassis,
    const tap::algorithms::odometry::ChassisWorldYawObserverInterface& yawObserver,
    float maxAcceleration)
    : chassis(chassis),
      yawObserver(yawObserver),
      maxAcceleration(maxAcceleration)
{
    assert(maxAcceleration > 0.0f);
}

inline void AccelerationLimitedOdometry2DTracker::updateVelocity(
    const modm::Matrix<float, 3, 1>& newVelocity3D,
    float chassisYaw)
{
    modm::Vector2f newVelocity2D = modm::Vector2f(newVelocity3D[0][0], newVelocity3D[1][0]);

    // Rotate new velocity into reference frame
    newVelocity2D.rotate(chassisYaw);

    // If this is first update, then just accept reported velocity
    if (prevVelocityUpdateTime == 0)
    {
        prevVelocityUpdateTime = tap::arch::clock::getTimeMilliseconds();
        velocity = newVelocity2D;
        return;
    }

    // Calculate maximum allowed delta velocity which is acceleration * dt
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevVelocityUpdateTime;
    prevVelocityUpdateTime = currTime;

    // m/s = m/s^2 * ms * 1s / 1000ms
    float maxDiff = maxAcceleration * static_cast<float>(dt) / 1'000.0f;

    modm::Vector2f deltaVelocity = newVelocity2D - velocity;

    float deltaMagnitude = deltaVelocity.getLength();

    if (deltaMagnitude > maxDiff)
    {
        // Change in velocity is greater than maximum allowed value. Scale delta
        // vector before applying to tracked velocity
        deltaVelocity *= maxDiff / deltaMagnitude;
    }

    velocity += deltaVelocity;
}

void AccelerationLimitedOdometry2DTracker::update()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevLocationUpdateTime;

    if (prevLocationUpdateTime == 0)
    {
        // This is first update. Bump time up to present then early return
        // to avoid inadequately long dt on first update
        prevLocationUpdateTime = currTime;
        return;
    }

    prevLocationUpdateTime = currTime;

    float chassisYaw;
    bool validOrientationAvailable = yawObserver.getChassisWorldYaw(&chassisYaw);

    if (validOrientationAvailable && chassis.allMotorsOnline())
    {
        modm::Matrix<float, 3, 1> newVelocity3D = chassis.getActualVelocityChassisRelative();
        // Make sure to do velocity updating first
        updateVelocity(newVelocity3D, chassisYaw);

        // m = m/s * 1 (s) / 1000 (ms)
        modm::Vector2f displacement = velocity * dt / 1000.0f;
        float newX = location.getX() + displacement.x;
        float newY = location.getY() + displacement.y;
        location.setPosition(newX, newY);
        location.setOrientation(chassisYaw);
    }
}

}  // namespace aruwsrc::algorithms::odometry
