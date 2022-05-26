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

#ifndef OTTO_VELOCITY_ODOMETRY_2D_SUBSYSTEM_HPP_
#define OTTO_VELOCITY_ODOMETRY_2D_SUBSYSTEM_HPP_

#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/algorithms/odometry/odometry_2d_tracker.hpp"
#include "tap/control/subsystem.hpp"

#include "modm/math/geometry/location_2d.hpp"

#include "chassis_kf_odometry.hpp"
#include "otto_chassis_velocity_displacement_2d_observer.hpp"
#include "otto_chassis_world_yaw_observer.hpp"

// Forward declarations
namespace aruwsrc
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
/**
 * @brief Velocity-based odometry class for the Otto vision system.
 *
 * Stores and tracks world position relative to arbitrary origin by integrating the velocity
 * reported by a ChassisSubsystem. The axes of the world frame are fixed to the field (ignoring
 * error) and are based on the axes used by the turret IMU (positive z-axis should be up and
 * out of field, x and y axes direction is undefined and just based on whatever the IMU uses).
 *
 * User is responsible for registering this subsystem with the command scheduler, or using some
 * other mechanism to call the `refresh` function periodically.
 *
 * A shallow inheritance of the tap::algorithms::odometry::Odometry2DInterface which just
 * simplifies the construction of the Otto chassis velocity and orientation getters.
 *
 * @see OttoChassisOrientationGetter
 * @see OttoChassisVelocityGetter
 */
class OttoVelocityOdometry2DSubsystem final : public tap::control::Subsystem,
                                              public ChassisKFOdometry
{
public:
    /**
     * @param[in] drivers pointer to aruwsrc drivers
     * @param[in] turret pointer to a TurretMotor object, @see OttoChassisWorldYawObserver for how
     * it is used
     * @param[in] chassis pointer to aruwsrc ChassisSubsystem
     */
    OttoVelocityOdometry2DSubsystem(
        aruwsrc::Drivers& drivers,
        const aruwsrc::control::turret::TurretSubsystem& turret,
        tap::control::chassis::ChassisSubsystemInterface& chassis);

    void refresh() override;

private:
    OttoChassisWorldYawObserver orientationObserver;
};

}  // namespace aruwsrc::algorithms::odometry

#endif  // OTTO_VELOCITY_ODOMETRY_2D_SUBSYSTEM_HPP_
