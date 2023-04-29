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

#ifndef SENTRY_OTTO_KF_ODOMETRY_2D_SUBSYSTEM_HPP_
#define SENTRY_OTTO_KF_ODOMETRY_2D_SUBSYSTEM_HPP_

#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/algorithms/odometry/odometry_2d_tracker.hpp"
#include "tap/control/subsystem.hpp"

#include "aruwsrc/algorithms/odometry/otto_chassis_world_yaw_observer.hpp"
#include "aruwsrc/algorithms/odometry/otto_kf_odometry_2d_subsystem.hpp"
#include "aruwsrc/control/chassis/sentry/sentry_drive_subsystem.hpp"
#include "modm/math/geometry/location_2d.hpp"

// #include "sentry_chassis_kf_odometry.hpp"


#include "sentry_kf_odometry.hpp"

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
/**
 * @brief Kalman Filter-based odometry class for the Otto vision system on the sentry.
 *
 * User is responsible for registering this subsystem with the command scheduler, or using some
 * other mechanism to call the `refresh` function periodically.
 *
 * @see ChassisKFOdometry
 */
class SentryOttoKFOdometry2DSubsystem final : public tap::control::Subsystem,
                                              public SentryKFOdometry
{
public:
    /**
     * @param[in] drivers reference to tap drivers
     * @param[in] chassis const reference to aruwsrc SentryDriveSubsystem
     * @param[in] turret const reference to a TurretMotor object, @see OttoChassisWorldYawObserver
     * for how it is used
     */
    SentryOttoKFOdometry2DSubsystem(
    tap::Drivers& drivers,
    aruwsrc::serial::VisionCoprocessor& visionCoprocessor,
    tap::communication::sensors::imu::ImuInterface& chassisIMU,
    const aruwsrc::chassis::HolonomicChassisSubsystem& chassis
    // const aruwsrc::control::turret::SentryTurretMajorSubsystem& turretMajor,
    // const aruwsrc::control::turret::SentryTurretMinorSubsystem& turretMinorLeft,
    // const aruwsrc::control::turret::SentryTurretMinorSubsystem& turretMinorRight
    );

    void refresh() override;

    // void initializeReferences(
    //     const aruwsrc::control::turret::SentryTurretMajorSubsystem& turretMajor,
    //     const aruwsrc::control::turret::SentryTurretMinorSubsystem& turretMinorLeft,
    //     const aruwsrc::control::turret::SentryTurretMinorSubsystem& turretMinorRight);
};

}  // namespace aruwsrc::algorithms::odometry

#endif  // SENTRY_OTTO_KF_ODOMETRY_2D_SUBSYSTEM_HPP_
