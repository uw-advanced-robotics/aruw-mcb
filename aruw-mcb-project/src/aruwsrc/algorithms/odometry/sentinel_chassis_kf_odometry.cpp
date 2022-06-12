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

#include "sentinel_chassis_kf_odometry.hpp"

#include "aruwsrc/control/sentinel/drive/sentinel_drive_subsystem.hpp"
#include "aruwsrc/drivers.hpp"

namespace aruwsrc::algorithms::odometry
{
SentinelChassisKFOdometry::SentinelChassisKFOdometry(
    aruwsrc::Drivers& drivers,
    const aruwsrc::control::sentinel::drive::SentinelDriveSubsystem& driveSubsystem,
    const aruwsrc::control::turret::TurretSubsystem& turret)
    : driveSubsystem(driveSubsystem),
      chassisYawObserver(&drivers, turret),
      imu(drivers.mpu6500),
      kf(KF_A, KF_C, KF_Q, KF_R, KF_P0)
//   chassisAccelerationToMeasurementCovarianceInterpolator(
//       CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT,
//       MODM_ARRAY_SIZE(CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT))
{
    float initialX[static_cast<int>(OdomState::NUM_STATES)] = {
        driveSubsystem.getAbsolutePosition(),
        0,
        0};
    kf.init(initialX);
}

void SentinelChassisKFOdometry::update()
{
    if (!chassisYawObserver.getChassisWorldYaw(&chassisYaw))
    {
        chassisYaw = 0;
    }

    // // Get chassis velocity as measured by the motor encoders.
    // // Since the sentinel chassis only moves in one direction,
    // // this measurement is in the world frame.
    // auto chassisVelocity = driveSubsystem.getActualVelocityChassisRelative();

    // // the measurement covariance is dynamically updated based on chassis-measured acceleration
    // updateMeasurementCovariance(chassisVelocity[0][1]);

    // assume 0 velocity/acceleration in z direction
    // float y[static_cast<int>(OdomInput::NUM_INPUTS)] = {};
    // Absolute position given in mm, so we should convert back to meters
    y[static_cast<int>(OdomInput::POS_Y)] = driveSubsystem.getAbsolutePosition() * 1E-3;
    y[static_cast<int>(OdomInput::VEL_Y)] = driveSubsystem.getActualVelocityChassisRelative()[1][0];
    y[static_cast<int>(OdomInput::ACC_Y)] = imu.getAy();

    // Update the Kalman filter. A new state estimate is available after this call.
    kf.performUpdate(y);

    // Update the location and velocity accessor objects with values from the state vector
    updateChassisStateFromKF();
}

void SentinelChassisKFOdometry::updateChassisStateFromKF()
{
    const auto& x = kf.getStateVectorAsMatrix();

    // update odometry velocity and orientation
    velocity.x = 0;
    velocity.y = x[static_cast<int>(OdomState::VEL_Y)];

    location.setOrientation(0);
    location.setPosition(0, x[static_cast<int>(OdomState::POS_Y)]);
}

// void SentinelChassisKFOdometry::updateMeasurementCovariance(const float& chassisVelocity)
// {
//     const uint32_t curTime = tap::arch::clock::getTimeMicroseconds();
//     const uint32_t dt = curTime - prevTime;
//     prevTime = curTime;

//     // return to avoid weird acceleration spike on startup
//     if (prevTime == 0)
//     {
//         return;
//     }

//     // compute acceleration

//     chassisMeasuredDeltaVelocity = tap::algorithms::lowPassFilter(
//         chassisMeasuredDeltaVelocity,
//         chassisVelocity - prevChassisVelocity,
//         CHASSIS_WHEEL_ACCELERATION_LOW_PASS_ALPHA);

//     prevChassisVelocity = chassisVelocity;

//     // dt is in microseconds, acceleration is dv / dt, so to get an acceleration with units
//     m/s^2,
//     // convert dt in microseconds to seconds
//     const float accelMagnitude = chassisMeasuredDeltaVelocity * 1E6 / static_cast<float>(dt);

//     const float velocityCovariance =
//         chassisAccelerationToMeasurementCovarianceInterpolator.interpolate(accelMagnitude);

//     // set measurement covariance of chassis velocity as measured by the wheels because if
//     // acceleration is large, the likelihood of slippage is greater
//     kf.getMeasurementCovariance()[0] = velocityCovariance;
//     kf.getMeasurementCovariance()[2 * static_cast<int>(OdomInput::NUM_INPUTS) + 2] =
//         velocityCovariance;
// }

}  // namespace aruwsrc::algorithms::odometry
