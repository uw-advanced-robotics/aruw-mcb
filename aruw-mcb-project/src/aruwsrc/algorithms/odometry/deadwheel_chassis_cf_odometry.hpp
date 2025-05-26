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

#ifndef DEADWHEEL_CHASSIS_CF_ODOMETRY_HPP_
#define DEADWHEEL_CHASSIS_CF_ODOMETRY_HPP_

#include "tap/algorithms/odometry/chassis_displacement_observer_interface.hpp"
#include "tap/algorithms/odometry/chassis_world_yaw_observer_interface.hpp"
#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"

#include "aruwsrc/algorithms/odometry/otto_chassis_world_yaw_observer.hpp"
#include "aruwsrc/algorithms/odometry/two_deadwheel_odometry_observer.hpp"
#include "modm/math/geometry/location_2d.hpp"

#include "two_deadwheel_odometry_observer.hpp"

namespace aruwsrc::algorithms::odometry
{
/**
 * An odometry interface that uses a complementary filter to measure odometry.
 */
class DeadwheelChassisCFOdometry : public tap::algorithms::odometry::Odometry2DInterface
{
    using DeadwheelOdometryObserver = aruwsrc::algorithms::odometry::TwoDeadwheelOdometryObserver;
    using YawObserver = tap::algorithms::odometry::ChassisWorldYawObserverInterface;
    using ImuInterface = tap::communication::sensors::imu::ImuInterface;

public:
    /**
     * Constructor.
     *
     * @param deadwheelOdometry The deadwheels of the robot for odometry measurements
     * @param chassisYawObserver Interface that computes the yaw of the chassis externally
     * @param imu IMU mounted on the chassis to measure chassis acceleration
     * @param initPos Initial position of chassis when robot boots
     * @param parallelCenterToWheelDistance Distance from the center of the chassis to the center of
     * the parallel deadwheel
     * @param parallelWheelChassisForwardRelativeAngleRadians Angle between the parallel deadwheel
     * and "forward" on the chassis
     * @param perpendicularWheelChassisForwardRelativeAngleRadians Angle between the perpendicular
     * deadwheel and "forward" on the chassis
     * @brief The parallel deadwheel is the deadwheel that is tangent to the edge of the chassis.
     * The perpendicular deadwheel is the deadwheel that is perpendicular to the edge of the
     * chassis. When moving in the direction of the parallel deadwheel, the perpendicular deadwheel
     * should not move, and vice versa
     */
    DeadwheelChassisCFOdometry(
        const DeadwheelOdometryObserver& deadwheelOdometry,
        YawObserver& chassisYawObserver,
        ImuInterface& imu,
        const modm::Vector2f initPos,
        const float parallelCenterToWheelDistance,
        const float parallelWheelChassisForwardRelativeAngleRadians,
        const float perpendicularWheelChassisForwardRelativeAngleRadians);

    inline modm::Location2D<float> getCurrentLocation2D() const final { return location; }

    inline modm::Vector2f getCurrentVelocity2D() const final { return velocity; }

    inline uint32_t getLastComputedOdometryTime() const final { return prevTime; }

    inline float getYaw() const override { return chassisYaw; }

    /**
     * @brief Resets the KF back to the robot's boot position.
     */
    void reset();

    void update();

    void overrideOdometryPosition(const float positionX, const float positionY)
    {
        location.setPosition(positionX, positionY);
    }

private:
    const DeadwheelOdometryObserver& deadwheelOdometry;
    YawObserver& chassisYawObserver;
    ImuInterface& imu;
    const modm::Vector2f initPos;

    /// Chassis location in the world frame
    modm::Location2D<float> location;
    /// Chassis velocity in the world frame
    modm::Vector2f velocity;
    // Chassis yaw orientation in world frame (radians)
    float chassisYaw = 0;

    /// Previous time `update` was called, in microseconds
    uint32_t prevTime = 0;

    float deadwheelTrust = 0.5f;  // Trust in deadwheel odometry vs IMU

    const float parallelCenterToWheelDistance;
    const float parallelWheelChassisForwardRelativeAngleRadians;
    const float perpendicularWheelChassisForwardRelativeAngleRadians;

    void computeDeadwheelVelocities(float* deadwheel_x_vel, float* deadwheel_y_vel) const;
    void computeAccVelocities(float* acc_x_vel, float* acc_y_vel, const float dt) const;
};
}  // namespace aruwsrc::algorithms::odometry

#endif  // CHASSIS_KF_ODOMETRY_HPP_
