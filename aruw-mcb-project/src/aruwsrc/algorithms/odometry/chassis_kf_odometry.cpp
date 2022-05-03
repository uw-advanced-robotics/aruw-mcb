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

#include "chassis_kf_odometry.hpp"

namespace aruwsrc::algorithms::odometry
{
ChassisKFOdometry::ChassisKFOdometry(
    const tap::control::chassis::ChassisSubsystemInterface& chassisSubsystem,
    tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
    tap::communication::sensors::imu::ImuInterface& imu)
    : chassisSubsystem(chassisSubsystem),
      chassisYawObserver(chassisYawObserver),
      imu(imu),
      kf(KF_A, KF_C, KF_Q, KF_R, KF_P),
      chassisPowerToSpeedInterpolator(
          CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT,
          MODM_ARRAY_SIZE(CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT))
{
    float initialX[static_cast<int>(OdomState::STATES)] = {};
    kf.init(initialX);
}

void ChassisKFOdometry::update()
{
    float chassisYaw = 0;
    if (!chassisYawObserver.getChassisWorldYaw(&chassisYaw))
    {
        return;
    }

    // get chassis relative velocity
    auto chassisVelocity = chassisSubsystem.getActualVelocityChassisRelative();
    tap::control::chassis::ChassisSubsystemInterface::getVelocityWorldRelative(
        chassisVelocity,
        chassisYaw);

    // the measurement covariance is dynamically updated based on chassis-measured acceleration
    updateMeasurementCovariance(chassisVelocity);

    // assume 0 velocity/acceleration in z direction
    float y[static_cast<int>(OdomInput::INPUTS)] = {};
    y[static_cast<int>(OdomInput::VEL_X)] = chassisVelocity[0][0];
    y[static_cast<int>(OdomInput::VEL_Y)] = chassisVelocity[1][0];
    // This is mounting-specific, the MCB is mounted in such a way that the accelerometer's Y
    // component corresponds to the X component and the X component corresponds to the chassis Y
    // acceleration component.
    y[static_cast<int>(OdomInput::ACC_X)] = imu.getAy();
    y[static_cast<int>(OdomInput::ACC_Y)] = imu.getAx();

    // acceleration in chassis frame, rotate to be in world frame
    tap::algorithms::rotateVector(
        &y[static_cast<int>(OdomInput::ACC_X)],
        &y[static_cast<int>(OdomInput::ACC_Y)],
        chassisYaw);

    // perform the update, new state matrix now available.
    kf.performUpdate(y);

    // update the location and velocity accessor objects with values from the state vector
    updateLocationVelocityFromKF(chassisYaw);
}

void ChassisKFOdometry::updateLocationVelocityFromKF(float chassisYaw)
{
    const auto& x = kf.getStateMatrix();

    // update odometry velocity and orientation
    velocity.x = x[static_cast<int>(OdomState::VEL_X)];
    velocity.y = x[static_cast<int>(OdomState::VEL_Y)];

    location.setOrientation(chassisYaw);
    location.setPosition(
        x[static_cast<int>(OdomState::POS_X)],
        x[static_cast<int>(OdomState::POS_Y)]);
}

void ChassisKFOdometry::updateMeasurementCovariance(
    const modm::Matrix<float, 3, 1>& chassisVelocity)
{
    const uint32_t curTime = tap::arch::clock::getTimeMicroseconds();
    const uint32_t dt = curTime - prevTime;
    prevTime = curTime;

    // return to avoid weird acceleration spike on startup
    if (prevTime == 0)
    {
        return;
    }

    // compute acceleration
    const modm::Matrix<float, 3, 1> deltaVelocity = chassisVelocity - prevChassisVelocity;
    prevChassisVelocity = chassisVelocity;

    const float xAccel = (deltaVelocity[0][0] * 1e6f) / static_cast<float>(dt);
    const float yAccel = (deltaVelocity[1][0] * 1e6f) / static_cast<float>(dt);
    const float accelMagnitude = hypot(xAccel, yAccel);

    const float velocityCovariance = chassisPowerToSpeedInterpolator.interpolate(accelMagnitude);

    // set measurement covariance of chassis velocity as measured by the wheels because if
    // acceleration is large, the likelihood of slippage is greater
    kf.getMeasurementCovariance()[0] = velocityCovariance;
    kf.getMeasurementCovariance()[2 * static_cast<int>(OdomInput::INPUTS) + 2] = velocityCovariance;
}

}  // namespace aruwsrc::algorithms::odometry
