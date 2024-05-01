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

#ifndef ODOMETRY_SUBSYSTEM_HPP_
#define ODOMETRY_SUBSYSTEM_HPP_

// #include "tap/algorithms/odometry/odometry_2d_interface.hpp"
// #include "tap/algorithms/odometry/odometry_2d_tracker.hpp"
#include "tap/control/subsystem.hpp"

#include "modm/math/geometry.hpp"

#include "chassis_kf_odometry.hpp"
#include "aruwsrc/algorithms/odometry/otto_chassis_world_yaw_observer.hpp"

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

namespace aruwsrc::chassis
{
/**
 * @brief Kalman Filter-based odometry class for the Otto vision system.
 *
 * User is responsible for registering this subsystem with the command scheduler, or using some
 * other mechanism to call the `refresh` function periodically.
 *
 * @see ChassisKFOdometry
 */
class OdometrySubsystem final : public tap::control::Subsystem, public ChassisKFOdometry
{
public:
    /**
     * @param[in] drivers pointer to aruwsrc drivers
     * @param[in] turret pointer to a TurretMotor object, @see OttoChassisWorldYawObserver for how
     * it is used
     * @param[in] chassis pointer to aruwsrc ChassisSubsystem
     * @param[in] initPos initial position of chassis on boot
     */
    OdometrySubsystem(
        tap::Drivers& drivers,
        const aruwsrc::control::turret::TurretSubsystem& turret,
        tap::control::chassis::ChassisSubsystemInterface& chassis,
        std::vector<aruwsrc::chassis::Wheel *> &wheels,
        modm::Vector2f initPos);

    void refresh() override;

private:
    aruwsrc::algorithms::odometry::OttoChassisWorldYawObserver orientationObserver;
};

}  // namespace aruwsrc::algorithms::odometry

#endif  // OTTO_KF_ODOMETRY_2D_SUBSYSTEM_HPP_
