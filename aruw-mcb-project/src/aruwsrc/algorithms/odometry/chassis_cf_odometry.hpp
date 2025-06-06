/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef CHASSIS_CF_ODOMETRY_HPP_
#define CHASSIS_CF_ODOMETRY_HPP_

#include "tap/algorithms/odometry/chassis_displacement_observer_interface.hpp"
#include "tap/algorithms/odometry/chassis_world_yaw_observer_interface.hpp"
#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"

#include "aruwsrc/algorithms/odometry/otto_chassis_world_yaw_observer.hpp"
#include "modm/math/geometry/location_2d.hpp"

namespace aruwsrc::algorithms::odometry
{
/**
 * An odometry interface that uses a complementary filter to measure odometry.
 */
class ChassisCFOdometry : public tap::algorithms::odometry::Odometry2DInterface,
                          public tap::control::Subsystem
{
public:
    /**
     * Constructor.
     *
     * @param chassisSubsystem The chassis subsystem of the robot for odometry measurements
     * @param chassisYawObserver Interface that computes the yaw of the chassis externally
     * @param imu IMU mounted on the chassis to measure chassis acceleration
     * @param initPos Initial position of chassis when robot boots
     */
    ChassisCFOdometry(
        tap::Drivers* drivers,
        const tap::control::chassis::ChassisSubsystemInterface& chassisSubsystem,
        tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
        tap::communication::sensors::imu::ImuInterface& imu,
        const modm::Vector2f initPos);

    inline modm::Location2D<float> getCurrentLocation2D() const final { return location; }

    inline modm::Vector2f getCurrentVelocity2D() const final { return velocity; }

    inline uint32_t getLastComputedOdometryTime() const final { return prevTime; }

    inline float getYaw() const override { return chassisYaw; }

    /**
     * @brief Resets the KF back to the robot's boot position.
     */
    void reset();

    void update();

    void refresh() override { update(); }

    void overrideOdometryPosition(const float positionX, const float positionY)
    {
        location.setPosition(positionX, positionY);
    }

private:
    const tap::control::chassis::ChassisSubsystemInterface& chassisSubsystem;
    tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver;
    tap::communication::sensors::imu::ImuInterface& imu;
    const modm::Vector2f initPos;

    /// Chassis location in the world frame
    modm::Location2D<float> location;
    /// Chassis velocity in the world frame
    modm::Vector2f velocity;
    // Chassis yaw orientation in world frame (radians)
    float chassisYaw = 0;

    /// Previous time `update` was called, in microseconds
    uint32_t prevTime = 0;

    float chassisTrust = 0.5f;  // Trust in deadwheel odometry vs IMU

    void computeAccVelocities(float* acc_x_vel, float* acc_y_vel, float dt);
};
}  // namespace aruwsrc::algorithms::odometry

#endif  // CHASSIS_CF_ODOMETRY_HPP_
