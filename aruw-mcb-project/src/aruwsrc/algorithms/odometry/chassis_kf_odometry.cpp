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

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"

namespace aruwsrc::algorithms::odometry
{
ChassisKFOdometry::ChassisKFOdometry(
    const tap::control::chassis::ChassisSubsystemInterface& chassisSubsystem,
    tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
    tap::communication::sensors::imu::ImuInterface& imu,
    const modm::Vector2f initPos)
    : kf(KF_A, KF_C, KF_Q, KF_R, KF_P0),
      chassisSubsystem(chassisSubsystem),
      chassisYawObserver(chassisYawObserver),
      imu(imu),
      initPos(initPos),
      chassisAccelerationToMeasurementCovarianceInterpolator(
          CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT,
          MODM_ARRAY_SIZE(CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT))
{
    reset();
}

void ChassisKFOdometry::reset()
{
    float initialX[int(OdomState::NUM_STATES)] = {initPos.x, 0.0f, 0.0f, initPos.y, 0.0f, 0.0f};
    kf.init(initialX);
}

void ChassisKFOdometry::update()
{
    if (!chassisYawObserver.getChassisWorldYaw(&chassisYaw))
    {
        chassisYaw = 0;
        return;
    }

    // get chassis frame velocity as measured by the motor encoders
    auto chassisVelocity = chassisSubsystem.getActualVelocityChassisRelative();
    tap::control::chassis::ChassisSubsystemInterface::getVelocityWorldRelative(
        chassisVelocity,
        chassisYaw);

    // the measurement covariance is dynamically updated based on chassis-measured acceleration
    updateMeasurementCovariance(chassisVelocity);

    // assume 0 velocity/acceleration in z direction
    float y[int(OdomInput::NUM_INPUTS)] = {};
    y[int(OdomInput::VEL_X)] = chassisVelocity[0][0];
    y[int(OdomInput::VEL_Y)] = chassisVelocity[1][0];
    y[int(OdomInput::ACC_X)] = imu.getAx();
    y[int(OdomInput::ACC_Y)] = imu.getAy();

    // rotate acceleration in MCB frame to the world frame
    tap::algorithms::rotateVector(
        &y[int(OdomInput::ACC_X)],
        &y[int(OdomInput::ACC_Y)],
        serial::VisionCoprocessor::MCB_ROTATION_OFFSET + chassisYaw);

    // perform the update, after this update a new state matrix is now available
    kf.performUpdate(y);

    // update the location and velocity accessor objects with values from the state vector
    updateChassisStateFromKF(chassisYaw);
}

void ChassisKFOdometry::updateChassisStateFromKF(float chassisYaw)
{

    // parallelWheelVelocity

    const auto& x = kf.getStateVectorAsMatrix();

    // update odometry velocity and orientation
    velocity.x = x[int(OdomState::VEL_X)];
    velocity.y = x[int(OdomState::VEL_Y)];

    location.setOrientation(chassisYaw);
    location.setPosition(x[int(OdomState::POS_X)], x[int(OdomState::POS_Y)]);
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

    chassisMeasuredDeltaVelocity.x = tap::algorithms::lowPassFilter(
        chassisMeasuredDeltaVelocity.x,
        chassisVelocity[0][0] - prevChassisVelocity[0][0],
        CHASSIS_WHEEL_ACCELERATION_LOW_PASS_ALPHA);

    chassisMeasuredDeltaVelocity.y = tap::algorithms::lowPassFilter(
        chassisMeasuredDeltaVelocity.y,
        chassisVelocity[1][0] - prevChassisVelocity[1][0],
        CHASSIS_WHEEL_ACCELERATION_LOW_PASS_ALPHA);

    prevChassisVelocity = chassisVelocity;

    // dt is in microseconds, acceleration is dv / dt, so to get an acceleration with units m/s^2,
    // convert dt in microseconds to seconds
    const float accelMagnitude =
        chassisMeasuredDeltaVelocity.getLength() * 1E6 / static_cast<float>(dt);

    const float velocityCovariance =
        chassisAccelerationToMeasurementCovarianceInterpolator.interpolate(accelMagnitude);

    // set measurement covariance of chassis velocity as measured by the wheels because if
    // acceleration is large, the likelihood of slippage is greater
    kf.getMeasurementCovariance()[0] = velocityCovariance;
    kf.getMeasurementCovariance()[2 * static_cast<int>(OdomInput::NUM_INPUTS) + 2] =
        velocityCovariance;
}

}  // namespace aruwsrc::algorithms::odometry
