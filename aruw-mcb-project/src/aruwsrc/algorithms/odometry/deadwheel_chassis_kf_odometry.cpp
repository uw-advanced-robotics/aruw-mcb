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

namespace aruwsrc::algorithms::odometry
{
DeadwheelChassisKFOdometry::DeadwheelChassisKFOdometry(
    const aruwsrc::algorithms::odometry::TwoDeadwheelOdometryObserver& deadwheelOdometry,
#if defined(TARGET_SENTRY_HYDRA)
    tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
#else
    aruwsrc::algorithms::odometry::OttoChassisWorldYawObserver& chassisYawObserver,
#endif
    tap::communication::sensors::imu::ImuInterface& imu,
    const modm::Vector2f initPos,
    const float parallelCenterToWheelDistance,
    const float parallelWheelChassisRelativeAngleRadians,
    const float perpendicularWheelChassisRelativeAngleRadians)
    : kf(KF_A, KF_C, KF_Q, KF_R, KF_P0),
      deadwheelOdometry(deadwheelOdometry),
      chassisYawObserver(chassisYawObserver),
      imu(imu),
      initPos(initPos),
      parallelCenterToWheelDistance(parallelCenterToWheelDistance),
      parallelWheelChassisRelativeAngleRadians(parallelWheelChassisRelativeAngleRadians),
      perpendicularWheelChassisRelativeAngleRadians(perpendicularWheelChassisRelativeAngleRadians)
{
    reset();
}

void DeadwheelChassisKFOdometry::reset()
{
    float initialX[int(OdomState::NUM_STATES)] = {initPos.x, 0.0f, 0.0f, initPos.y, 0.0f, 0.0f};
    kf.init(initialX);
}

void DeadwheelChassisKFOdometry::update()
{
    if (!chassisYawObserver.getChassisWorldYaw(&chassisYaw))
    {
        chassisYaw = 0;
        return;
    }

    // Assuming getPerpendicularWheelVelocity() and getParallelWheelVelocity() return the velocities
    // of the two omni wheels
    perpendicularRaw = deadwheelOdometry.getPerpendicularVelocity();
    parallelRaw = deadwheelOdometry.getParallelMotorVelocity();

    // Calculate velocities in the robot's frame of reference
    // Correct for rotation of the robot
    parallelRaw -= imu.getGz() * parallelCenterToWheelDistance;
    
    // Rotate the velocities based on the wheel rotations
    float Vx = parallelRaw * parallelWheelChassisRelativeAngleRadians;
    float Vy = perpendicularRaw * perpendicularWheelChassisRelativeAngleRadians;

    tap::algorithms::rotateVector(&Vx, &Vy, chassisYaw);

    // Get acceleration from IMU
    float ax = imu.getAx();
    float ay = imu.getAy();

    // Rotate acceleration to the world frame
    float accelXWorld, accelYWorld;
    tap::algorithms::rotateVector(&ax, &ay, chassisYaw);
    accelXWorld = ax;
    accelYWorld = ay;

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
    prevTime = tap::arch::clock::getTimeMicroseconds();
}

void DeadwheelChassisKFOdometry::overrideOdometryPosition(
    const float positionX,
    const float positionY)
{
    auto currKFState = kf.getStateVectorAsMatrix();

    float newState[int(OdomState::NUM_STATES)] = {
        positionX,
        currKFState[int(OdomState::VEL_X)],
        currKFState[int(OdomState::ACC_X)],
        positionY,
        currKFState[int(OdomState::VEL_Y)],
        currKFState[int(OdomState::ACC_Y)]};

    kf.init(newState);
}

}  // namespace aruwsrc::algorithms::odometry
