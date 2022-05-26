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

namespace aruwsrc::algorithms::odometry
{
SentinelChassisKFOdometry::SentinelChassisKFOdometry(
    const tap::control::chassis::ChassisSubsystemInterface& chassisSubsystem,
    tap::communication::sensors::imu::ImuInterface& imu)
    : chassisSubsystem(chassisSubsystem),
      imu(imu),
      kf(KF_A, KF_C, KF_Q, KF_R, KF_P),
      chassisAccelerationToCovarianceInterpolator(
          CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT,
          MODM_ARRAY_SIZE(CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT))
{
    float initialX[static_cast<int>(OdomState::STATES)] = {};
    kf.init(initialX);
}

void SentinelChassisKFOdometry::update()
{
    // get chassis relative velocity
    auto chassisVelocity = chassisSubsystem.getActualVelocityChassisRelative();

    // the measurement covariance is dynamically updated based on chassis-measured acceleration
    updateMeasurementCovariance(chassisVelocity[0][1]);

    // assume 0 velocity/acceleration in z direction
    float y[static_cast<int>(OdomInput::INPUTS)] = {};
    y[static_cast<int>(OdomInput::POS_Y)] = chassisVelocity[0][1];
    // This is mounting-specific, the MCB is mounted in such a way that the accelerometer's Y
    // component corresponds to the X component and the X component corresponds to the chassis Y
    // acceleration component.
    y[static_cast<int>(OdomInput::ACC_Y)] = imu.getAy();

    // perform the update, new state matrix now available.
    kf.performUpdate(y);

    // update the location and velocity accessor objects with values from the state vector
    updateLocationVelocityFromKF();
}

void SentinelChassisKFOdometry::updateLocationVelocityFromKF()
{
    const auto& x = kf.getStateMatrix();

    // update odometry velocity and orientation
    velocity.x = 0;
    velocity.y = x[static_cast<int>(OdomState::VEL_Y)];

    location.setOrientation(0);
    location.setPosition(
        0,
        x[static_cast<int>(OdomState::POS_Y)]);
}

void SentinelChassisKFOdometry::updateMeasurementCovariance(
    const float& chassisVelocity)
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

    deltaVelocity = tap::algorithms::lowPassFilter(
        deltaVelocity,
        chassisVelocity - prevChassisVelocity,
        CHASSIS_WHEEL_ACCELERATION_LOW_PASS_ALPHA);

    prevChassisVelocity = chassisVelocity;

    const float accelMagnitude = deltaVelocity * 1E6 / static_cast<float>(dt);

    const float velocityCovariance = chassisAccelerationToCovarianceInterpolator.interpolate(accelMagnitude);

    // set measurement covariance of chassis velocity as measured by the wheels because if
    // acceleration is large, the likelihood of slippage is greater
    kf.getMeasurementCovariance()[0] = velocityCovariance;
    kf.getMeasurementCovariance()[2 * static_cast<int>(OdomInput::INPUTS) + 2] = velocityCovariance;
}

}  // namespace aruwsrc::algorithms::odometry
