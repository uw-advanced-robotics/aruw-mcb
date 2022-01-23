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

#ifndef OTTO_MECANUM_VELOCITY_ODOMETRY_SUBSYSTEM_HPP_
#define OTTO_MECANUM_VELOCITY_ODOMETRY_SUBSYSTEM_HPP_

#include "tap/control/odometry/odometry_subsystem.hpp"
#include "otto_chassis_orientation_getter.hpp"
#include "otto_chassis_velocity_displacement_getter.hpp"

// Forward declarations
namespace aruwsrc
{
class Drivers;
}
namespace tap::control::turret
{
class TurretSubsystemInterface;
}
namespace aruwsrc::chassis
{
class ChassisSubsystem;
}

namespace aruwsrc::control::odometry
{
/**
 * @brief Velocity-based odometry class for the Otto vision system. 
 * 
 * Stores and tracks world position relative to arbitrary origin by integrating the velocity
 * reported by a ChassisSubsystem. The axes of the world frame are fixed to the field (ignoring
 * error) and are based on the axes used by the turret IMU (positive z-axis should be up and
 * out of field, x and y axes direction is undefined and just based on whatever the IMU uses).
 * 
 * User is responsible for registering this subsystem with the command scheduler.
 * 
 * A shallow inheritance of the tap::control::odometry::VelocityOdometrySubsystem which just
 * simplifies the construction of the Otto chassis velocity and orientation getters.
 * 
 * @see tap::control::odometry::VelocityOdometrySubsystem
 * @see OttoChassisOrientationGetter
 * @see OttoChassisVelocityGetter
 */
class OttoVelocityOdometrySubsystem final : public tap::control::odometry::OdometrySubsystem
{
public:
    /**
     * @param[in] drivers pointer to aruwsrc drivers
     * @param[in] turret pointer to object that implements TurretSubsystemInterface
     * @param[in] chassis pointer to aruwsrc ChassisSubsystem
     */
    OttoVelocityOdometrySubsystem(
        aruwsrc::Drivers* drivers,
        tap::control::turret::TurretSubsystemInterface* turret, 
        aruwsrc::chassis::ChassisSubsystem* chassis);

private:
    tap::Drivers* drivers;
    OttoChassisOrientationGetter orientationGetter;
    OttoChassisVelocityDisplacementGetter displacementGetter;
};

}  // namespace aruwsrc::control::odometry

#endif  // OTTO_MECANUM_VELOCITY_ODOMETRY_SUBSYSTEM_HPP_
