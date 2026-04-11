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

#ifndef THREE_DEADWHEEL_KF_ODOMETRY_2D_SUBSYSTEM_HPP_
#define THREE_DEADWHEEL_KF_ODOMETRY_2D_SUBSYSTEM_HPP_

#include "tap/algorithms/odometry/chassis_world_yaw_observer_interface.hpp"
#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/algorithms/odometry/odometry_2d_tracker.hpp"
#include "tap/control/subsystem.hpp"

#include "aruwsrc/algorithms/odometry/otto_chassis_world_yaw_observer.hpp"
#include "aruwsrc/algorithms/odometry/three_deadwheel_chassis_kf_odometry.hpp"
#include "aruwsrc/algorithms/odometry/three_deadwheel_odometry_observer.hpp"
#include "modm/math/geometry/location_2d.hpp"
#include "modm/math/geometry/vector2.hpp"

// Forward declarations
namespace tap
{
class Drivers;
}
namespace aruwsrc::control::turret
{
class TurretSubsystem;
}

namespace tap::control::chassis
{
class ChassisSubsystemInterface;
}

namespace aruwsrc::algorithms::odometry
{
class ThreeDeadwheelKFOdometry2DSubsystem
    : public tap::control::Subsystem,
      public aruwsrc::algorithms::odometry::ThreeDeadwheelChassisKFOdometry
{
public:
    /**
     * @brief Kalman Filter-based odometry class for the Otto vision system on the sentry.
     *
     * User is responsible for registering this subsystem with the command scheduler, or using some
     * other mechanism to call the `refresh` function periodically.
     *
     * @see ChassisKFOdometry
     *
     * @param[in] drivers reference to tap drivers
     * @param[in] deadwheelOdometry reference to deadwheels for odometry data
     * @param[in] yawObserver reference to a SentryChassisWorldYawObserver, which provides world
     * relative yaw of the chassis @see OttoChassisWorldYawObserver for how it is used
     * @param[in] imu reference to the chassis-mounted IMU
     * @param[in] initialXPos initial world-frame x position of the chassis
     * @param[in] initialYPos initial world-frame y position of the chassis
     */
    ThreeDeadwheelKFOdometry2DSubsystem(
        tap::Drivers &drivers,
        const aruwsrc::algorithms::odometry::ThreeDeadwheelOdometryObserver &deadwheels,
#if defined(TARGET_SENTRY_ECLIPSE)
        tap::algorithms::odometry::ChassisWorldYawObserverInterface &yawObserver,
#else
        const aruwsrc::control::turret::TurretSubsystem &yawObserver,
#endif
        tap::communication::sensors::imu::ImuInterface &imu,
        float initialXPos,
        float initialYPos,
        float initialYaw,
        const float parallelOneCenterToWheelDistance,
        const float parallelTwoCenterToWheelDistance,
        const float perpendicularCenterToWheelDistance,
        const float odomFrameToRobotFrame);

    void refresh() override;

    void overrideOdometryPosition(const float positionX, const float positionY);

    void overrideOdometryOrientation(const float deltaYaw);

private:
#if defined(TARGET_SENTRY_ECLIPSE)
    tap::algorithms::odometry::ChassisWorldYawObserverInterface &chassisYawObserver;
#else
    aruwsrc::algorithms::odometry::OttoChassisWorldYawObserver chassisYawObserver;
#endif
};

}  // namespace aruwsrc::algorithms::odometry

#endif  // THREE_DEADWHEEL_KF_ODOMETRY_2D_SUBSYSTEM_HPP_
