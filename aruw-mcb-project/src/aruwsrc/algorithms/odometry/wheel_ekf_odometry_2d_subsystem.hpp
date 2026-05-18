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

#ifndef WHEEL_EKF_ODOMETRY_2D_SUBSYSTEM_HPP_
#define WHEEL_EKF_ODOMETRY_2D_SUBSYSTEM_HPP_

#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/algorithms/odometry/odometry_2d_tracker.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"
#include "tap/control/subsystem.hpp"

#include "modm/math/geometry.hpp"

#include "wheel_ekf_odometry.hpp"

// Forward declarations
namespace tap
{
class Drivers;
namespace motor
{
class DjiMotor;
}
}  // namespace tap
namespace aruwsrc::communication::rtt
{
class RttTelemetry;
}
namespace aruwsrc::algorithms::odometry
{
/**
 * @brief Kalman Filter-based odometry class using individual wheel velocities.
 *
 * User is responsible for registering this subsystem with the command scheduler, or using some
 * other mechanism to call the `refresh` function periodically.
 *
 * @see FourWheelKFOdometry
 */
class WheelEKFOdometry2DSubsystem final : public tap::control::Subsystem,
                                          public FourWheelEKFOdometry
{
public:
    /**
     * @param[in] drivers pointer to aruwsrc drivers
     * @param[in] chassisMotors array of 4 chassis motor pointers
     * @param[in] yawObserver yaw observer used to provide world-frame yaw measurements
     * @param[in] imu IMU source used for acceleration and yaw-rate measurements
     * @param[in] initPos initial position of chassis on boot
     */
    WheelEKFOdometry2DSubsystem(
        tap::Drivers& drivers,
        const tap::motor::DjiMotor* chassisMotors[4],
        tap::algorithms::odometry::ChassisWorldYawObserverInterface& yawObserver,
        tap::communication::sensors::imu::ImuInterface& imu,
        const modm::Vector2f initPos,
        aruwsrc::communication::rtt::RttTelemetry* telemetry = nullptr);

    void refresh() override;
};

}  // namespace aruwsrc::algorithms::odometry

#endif  // WHEEL_EKF_ODOMETRY_2D_SUBSYSTEM_HPP_
