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

#ifndef SENTRY_KF_ODOMETRY_2D_SUBSYSTEM_HPP_
#define SENTRY_KF_ODOMETRY_2D_SUBSYSTEM_HPP_

#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/algorithms/odometry/odometry_2d_tracker.hpp"
#include "tap/control/subsystem.hpp"

#include "aruwsrc/algorithms/odometry/chassis_kf_odometry.hpp"
#include "aruwsrc/robot/sentry/sentry_chassis_world_yaw_observer.hpp"
#include "aruwsrc/robot/sentry/sentry_kf_odometry_2d_subsystem.hpp"
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

namespace aruwsrc::sentry
{
class SentryKFOdometry2DSubsystem : public tap::control::Subsystem,
                                    public aruwsrc::algorithms::odometry::ChassisKFOdometry
{
public:
    // @note: should add an imuToChassisCenter vector to transform
    // readings (otherwise readings will just be bad :O)
    SentryKFOdometry2DSubsystem(
        tap::Drivers &drivers,
        tap::control::chassis::ChassisSubsystemInterface &chassis,
        SentryChassisWorldYawObserver &yawObserver,
        tap::communication::sensors::imu::ImuInterface &imu,
        float initialXPos,
        float initialYPos);

    void refresh() override;

    void overrideOdometryPosition(const modm::Vector2f &newPos);

    void overrideOdometryOrientation(const float deltaYaw);
};

}  // namespace aruwsrc::sentry

#endif  // SENTRY_KF_ODOMETRY_2D_SUBSYSTEM_HPP_
