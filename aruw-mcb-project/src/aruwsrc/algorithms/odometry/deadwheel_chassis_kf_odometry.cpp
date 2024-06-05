/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "deadwheel_chassis_kf_odometry.hpp"
#include <queue>
#include "aruwsrc/algorithms/moving_average.hpp"

namespace aruwsrc::algorithms::odometry
{
DeadwheelChassisKFOdometry::DeadwheelChassisKFOdometry(
    const aruwsrc::algorithms::odometry::TwoDeadwheelOdometryObserver& deadwheelOdometry,
    tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
    tap::communication::sensors::imu::ImuInterface& imu,
    const modm::Vector2f initPos,
    const float centerToWheelDistance)
    : kf(KF_A, KF_C, KF_Q, KF_R, KF_P0),
      deadwheelOdometry(deadwheelOdometry),
      chassisYawObserver(chassisYawObserver),
      imu(imu),
      initPos(initPos),
      chassisAccelerationToMeasurementCovarianceInterpolator(
          CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT,
          MODM_ARRAY_SIZE(CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT)),
      centerToWheelDistance(centerToWheelDistance)
{
    reset();
}

void DeadwheelChassisKFOdometry::reset()
{
    float initialX[int(OdomState::NUM_STATES)] = {initPos.x, 0.0f, 0.0f, initPos.y, 0.0f, 0.0f};
    kf.init(initialX);
}

float Vxd, Vyd;
float V1d, V2d;
float alpha = 0.001;
std::queue<float> V1s, V2s;
size_t windowPeriod = 5;
float sum1, sum2;
MovingAverage V1avg(windowPeriod, 0), V2avg(windowPeriod, 0);
void DeadwheelChassisKFOdometry::update()
{
    if (!chassisYawObserver.getChassisWorldYaw(&chassisYaw))
    {
        chassisYaw = 0;
        return;
    }

    // Assuming getPerpendicularWheelVelocity() and getParallelWheelVelocity() return the velocities
    // of the two omni wheels
    float fakeV1 = deadwheelOdometry.getPerpendicularRPM();
    float fakeV2 = deadwheelOdometry.getParallelMotorRPM();
    fakeV1 = deadwheelOdometry.rpmToMetersPerSecond(fakeV1);
    fakeV2 = deadwheelOdometry.rpmToMetersPerSecond(fakeV2);
    V1avg.update(fakeV1);
    V2avg.update(fakeV2);
    float V1 = V1avg.getVal();
    float V2 = V2avg.getVal();
    V1d = V1;
    V2d = V2;
    // sum1 += fakeV1;
    // sum2 += fakeV2;
    // V1s.push(fakeV1);
    // V2s.push(fakeV2);
    // if (V1s.size() > windowPeriod)
    // {
    //     sum1 -= V1s.front();
    //     V1s.pop();
    //     sum2 -= V2s.front();
    //     V2s.pop();
    // }
    // // V1d += alpha * (fakeV1 - V1d);
    // // V2d += alpha * (fakeV2 - V2d);
    // V1d = sum1 / windowPeriod;
    // V2d = sum2 / windowPeriod;
    // float V1 = V1d;
    // float V2 = V2d;
    // Calculate velocities in the robot's frame of reference
    // Correct for roation of the robot
    V2 -= modm::toRadian(imu.getGz()) * centerToWheelDistance;
    // It is assumed that the wheels are rotated 45 degrees
    // relative to the forward direction of the robot
    float Vx = (((V1 - V2)) / M_SQRT2);
    float Vy = (((V1 + V2)) / M_SQRT2);
    Vxd = Vx;
    Vyd = Vy;
    tap::algorithms::rotateVector(&Vx, &Vy, chassisYaw);
    // Get acceleration from IMU
    float ax = imu.getAx();
    float ay = imu.getAy();

    // Rotate acceleration to the world frame
    float accelXWorld, accelYWorld;
    tap::algorithms::rotateVector(&ax, &ay, chassisYaw);
    accelXWorld = ax;
    accelYWorld = ay;

    // The measurement covariance is dynamically updated based on chassis-measured acceleration
    updateMeasurementCovariance(Vx, Vy);

    // Create the measurement vector
    float y[int(OdomInput::NUM_INPUTS)] = {Vx, accelXWorld, Vy, accelYWorld};

    // Perform the Kalman filter update
    kf.performUpdate(y);
    updateChassisStateFromKF(chassisYaw);
}

void DeadwheelChassisKFOdometry::updateChassisStateFromKF(float chassisYaw)
{
    const auto& x = kf.getStateVectorAsMatrix();

    // update odometry velocity and orientation
    velocity.x = x[int(OdomState::VEL_X)];
    velocity.y = x[int(OdomState::VEL_Y)];

    location.setOrientation(chassisYaw);
    location.setPosition(x[int(OdomState::POS_X)], x[int(OdomState::POS_Y)]);
}

void DeadwheelChassisKFOdometry::updateMeasurementCovariance(float Vx, float Vy)
{
    const uint32_t curTime = tap::arch::clock::getTimeMicroseconds();
    const uint32_t dt = curTime - prevTime;
    prevTime = curTime;

    // Return to avoid weird acceleration spike on startup
    if (prevTime == 0)
    {
        return;
    }

    // Compute acceleration
    chassisMeasuredDeltaVelocity.x = tap::algorithms::lowPassFilter(
        chassisMeasuredDeltaVelocity.x,
        Vx - prevChassisVelocity[0][0],
        CHASSIS_WHEEL_ACCELERATION_LOW_PASS_ALPHA);

    chassisMeasuredDeltaVelocity.y = tap::algorithms::lowPassFilter(
        chassisMeasuredDeltaVelocity.y,
        Vy - prevChassisVelocity[1][0],
        CHASSIS_WHEEL_ACCELERATION_LOW_PASS_ALPHA);

    prevChassisVelocity[0][0] = Vx;
    prevChassisVelocity[1][0] = Vy;

    // dt is in microseconds, acceleration is dv / dt, so to get an acceleration with units m/s^2,
    // convert dt in microseconds to seconds
    const float accelMagnitude =
        chassisMeasuredDeltaVelocity.getLength() * 1E6 / static_cast<float>(dt);

    const float velocityCovariance =
        chassisAccelerationToMeasurementCovarianceInterpolator.interpolate(accelMagnitude);

    // Set measurement covariance of chassis velocity as measured by the wheels
    kf.getMeasurementCovariance()[0] = velocityCovariance;
    kf.getMeasurementCovariance()[2 * static_cast<int>(OdomInput::NUM_INPUTS) + 2] =
        velocityCovariance;
}

}  // namespace aruwsrc::algorithms::odometry
