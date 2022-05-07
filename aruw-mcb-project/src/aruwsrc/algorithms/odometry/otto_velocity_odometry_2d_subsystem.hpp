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

#include "otto_chassis_velocity_displacement_2d_observer.hpp"
#include "otto_chassis_world_yaw_observer.hpp"

// Forward declarations
namespace aruwsrc
{
class Drivers;
}
namespace aruwsrc::control::turret
{
class TurretMotor;
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
                                              public tap::algorithms::odometry::Odometry2DInterface
{
public:
    /**
     * @param[in] drivers pointer to aruwsrc drivers
     * @param[in] turret pointer to a TurretMotor object, @see OttoChassisWorldYawObserver for how
     * it is used
     * @param[in] chassis pointer to aruwsrc ChassisSubsystem
     */
    OttoVelocityOdometry2DSubsystem(
        aruwsrc::Drivers* drivers,
        const aruwsrc::control::turret::TurretMotor* turret,
        tap::control::chassis::ChassisSubsystemInterface* chassis);

    void refresh() override;

    modm::Location2D<float> getCurrentLocation2D() const override final
    {
        return odometryTracker.getCurrentLocation2D();
    }

    modm::Vector2f getCurrentVelocity2D() const override final
    {
        return odometryTracker.getCurrentVelocity2D();
    }

    float getYaw() const override final
    {
        return odometryTracker.getYaw();
    }

    /**
     * @brief Use a given turret origin with the current robot location
     * to get the location of the turret in the world frame.
     * 
     * @param turretOrigin in the robot frame.
     * 
     * @return The current turret location in the world frame.
     */
    modm::Vector3f getCurrentTurretLocation(modm::Vector3f turretOrigin)
    {
        return yawRotation*turretOrigin + modm::Vector3f(getCurrentLocation2D().getPosition(), 0);
    }

private:

    modm::Matrix3f rotationMatrixXY(float yaw)
    {
        float cosYaw = cosf(yaw);
        float sinYaw = sin(yaw);
        const float m[9] = {cosYaw, -sinYaw, 0,
                      sinYaw, cosYaw, 0,
                      0, 0, 1};
        return modm::Matrix3f(m);
    }

    tap::algorithms::odometry::Odometry2DTracker odometryTracker;
    OttoChassisWorldYawObserver orientationObserver;
    OttoChassisVelocityDisplacement2DObserver displacementObserver;
};

}  // namespace aruwsrc::algorithms::odometry

#endif  // OTTO_VELOCITY_ODOMETRY_2D_SUBSYSTEM_HPP_
