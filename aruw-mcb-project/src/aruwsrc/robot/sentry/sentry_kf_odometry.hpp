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

#ifndef SENTRY_KF_ODOMETRY_HPP_
#define SENTRY_KF_ODOMETRY_HPP_

#include "tap/algorithms/kalman_filter.hpp"
#include "tap/algorithms/odometry/chassis_displacement_observer_interface.hpp"
#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"

#include "aruwsrc/algorithms/odometry/otto_chassis_world_yaw_observer.hpp"
#include "aruwsrc/control/chassis/sentry/sentry_drive_subsystem.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "modm/math/geometry/location_2d.hpp"
#include "modm/math/interpolation/linear.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"

namespace aruwsrc::algorithms::odometry
{
/**
 * An odometry interface that uses a kalman filter to measure odometry. This class is designed
 * specifically for the sentry.
 *
 * @note Assumes the world frame has an origin of (0, 0) wherever the robot was booted from.
 */
class SentryKFOdometry : public tap::algorithms::odometry::Odometry2DInterface
{
public:
    /**
     * @param[in] drivers: A reference to the drivers object. Used to obtain chassis IMU
     * measurements from the mpu6500 onboard the MCB, as well as to construct an
     * `OttoChassisWorldYawObserver` so that this class can be an `Odometry2DInterface`.
     * @param[in] chassis: A const reference to the sentry drive subsystem. Used to fetch
     * velocity and absolute position measurements from the encoders (and limit switches for the
     * latter).
     * @param[in] turret: A const reference to the turret subsystem from which odometry should be
     * tracked. In general, the only purpose of this is to produce yaw measurements, which for this
     * implementation does not affect the calculated kinematic state of the sentry chassis.
     * However, it is needed for inheritance since an `OttoChassisWorldYawObserver` is needed to
     * qualify this class as an `Odometry2DInterface`.
     */
    SentryKFOdometry(
        tap::Drivers& drivers,
        const aruwsrc::control::sentry::drive::SentryDriveSubsystem& chassis,
        const aruwsrc::control::turret::TurretSubsystem& turret);

    virtual modm::Location2D<float>  getCurrentLocation2D() const override;

    virtual modm::Vector2f getCurrentVelocity2D() const override;

    virtual uint32_t getLastComputedOdometryTime() const override;

    virtual float getYaw() const override;
    
    // Kalman Filter enums
    // Chassis Inputs
    enum class OdomInput {
        VEL_X = 0,
        ACC_X,
        VEL_Y,
        ACC_Y,
        POS_YAW,
        VEL_YAW,
        NUM_INPUTS
    };

    enum class OdomState {
        POS_X = 0,
        VEL_X,
        ACC_X,
        POS_Y,
        VEL_Y,
        ACC_Y,
        POS_YAW,
        VEL_YAW,
        ACC_YAW,
        NUM_STATES
    };
};
} // namespace namespace aruwsrc::algorithms::odometry

#endif // SENTRY_KF_ODOMETRY_HPP_
