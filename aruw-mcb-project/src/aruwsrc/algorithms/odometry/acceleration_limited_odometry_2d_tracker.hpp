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

#ifndef ACCELERATION_LIMITED_ODOMETRY_2D_TRACKER_HPP_
#define ACCELERATION_LIMITED_ODOMETRY_2D_TRACKER_HPP_

#include <cstdint>

#include "tap/algorithms/odometry/chassis_displacement_observer_interface.hpp"
#include "tap/algorithms/odometry/chassis_world_yaw_observer_interface.hpp"
#include "tap/algorithms/odometry/odometry_2d_interface.hpp"

#include "modm/math/geometry/location_2d.hpp"
#include "modm/math/geometry/vector.hpp"

// Forward declarations
namespace tap::control::chassis
{
class ChassisSubsystemInterface;
}

namespace aruwsrc::algorithms::odometry
{
/**
 * @brief Odometry2D tracker that accounts for slippage by limiting maximum measured acceleration
 *
 * @see tap::algorithms::odometry::Odometry2DInterface
 */
class AccelerationLimitedOdometry2DTracker : public tap::algorithms::odometry::Odometry2DInterface
{
public:
    /**
     * @param[in] chassis reference to a chassis object. Used to get chassis-frame velocity
     * @param[in] yawObserver reference to a yaw observer measuring the world yaw of the chassis.
     * This defines the orientation of the coordinate frame which this tracker will measure odometry
     * relative to
     * @param[in] maxAcceleration the max acceleration in m/s^2 that the chassis is capable of even
     * when slipping
     */
    AccelerationLimitedOdometry2DTracker(
        const tap::control::chassis::ChassisSubsystemInterface& chassis,
        const tap::algorithms::odometry::ChassisWorldYawObserverInterface& yawObserver,
        float maxAcceleration);

    /**
     * Read-in the most recent velocity and orientation data and update the stored odometry
     * location. Call frequently for better results.
     */
    void update();

    modm::Location2D<float> getCurrentLocation2D() const final { return location; }

    modm::Vector2f getCurrentVelocity2D() const final { return velocity; }

    virtual float getYaw() const
    {
        float yaw;
        if (!yawObserver.getChassisWorldYaw(&yaw))
        {
            return 0;
        }
        else
        {
            return yaw;
        }
    }

    virtual uint32_t getLastComputedOdometryTime() const { return prevLocationUpdateTime; }

private:
    /**
     * Update velocity to match new value, limited to a max acceleration.
     *
     * @param[in] newVelocity3D most recent chassis-relative velocity reported by the chassis
     * @param[in] chassisYaw the most recently reported yaw
     *
     * @note this function is updates a previous time tracker
     */
    void updateVelocity(const modm::Matrix<float, 3, 1>& newVelocity3D, float chassisYaw);

    /**
     * Chassis object which this tracker is measuring velocity from.
     */
    const tap::control::chassis::ChassisSubsystemInterface& chassis;

    /**
     * Yaw observer measuring orientation of the chassis in world frame.
     */
    const tap::algorithms::odometry::ChassisWorldYawObserverInterface& yawObserver;

    /**
     * Stores position and orientation this object believes the chassis to be at in reference frame.
     *
     * Position in meters, orientation in radians.
     *
     * Origin of position starts where chassis starts. 0-angle of orientation and positive
     * direction of rotation are yawObserver defined.
     */
    modm::Location2D<float> location;

    /**
     * Velocity in 2D in m/s in reference frame. Positive x and y are forward and left respectively.
     */
    modm::Vector2f velocity;

    /**
     * Maximum acceleration this tracker allows in m/s^2
     */
    const float maxAcceleration;

    /**
     * Previous time at which this->location was updated in ms. 0 when no valid data was available
     * last update.
     */
    uint32_t prevLocationUpdateTime = 0;

    /**
     * Previous time at which velocity was updated in ms. Initialized to 0
     */
    uint32_t prevVelocityUpdateTime = 0;
};

}  // namespace aruwsrc::algorithms::odometry

#endif  // ACCELERATION_LIMITED_ODOMETRY_2D_TRACKER_HPP_
