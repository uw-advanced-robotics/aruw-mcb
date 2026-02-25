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

#include "three_deadwheel_chassis_kf_odometry.hpp"

namespace aruwsrc::algorithms::odometry
{
using namespace tap::algorithms;

ThreeDeadwheelChassisKFOdometry::ThreeDeadwheelChassisKFOdometry(
    const aruwsrc::algorithms::odometry::ThreeDeadwheelOdometryObserver& deadwheelOdometry,
#if defined(TARGET_SENTRY_NAME)
    tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
#else
    aruwsrc::algorithms::odometry::OttoChassisWorldYawObserver& chassisYawObserver,
#endif
    tap::communication::sensors::imu::ImuInterface& imu,
    const modm::Vector2f initPos,
    const float initYaw,
    const float parallelOneCenterToWheelDistance,
    const float parallelTwoCenterToWheelDistance,
    const float perpendicularCenterToWheelDistance,
    const float odomFrameToRobotFrame)
    : kf(KF_A, KF_C, KF_Q, KF_R, KF_P0),
      deadwheelOdometry(deadwheelOdometry),
      chassisYawObserver(chassisYawObserver),
      imu(imu),
      initPos(initPos),
      initYaw(initYaw),
      chassisYaw(initYaw),
      parallelOneCenterToWheelDistance(parallelOneCenterToWheelDistance),
      parallelTwoCenterToWheelDistance(parallelTwoCenterToWheelDistance),
      perpendicularCenterToWheelDistance(perpendicularCenterToWheelDistance),
      odomFrameToRobotFrame(odomFrameToRobotFrame)
{
    reset();
}

void ThreeDeadwheelChassisKFOdometry::reset()
{
    chassisYaw = tap::algorithms::Angle(initYaw);
    imuTheta = 0.0f;
    lastWrappedTheta = 0.0f;

    float initialX[int(OdomState::NUM_STATES)] =
        {initPos.x, 0.0f, 0.0f, initPos.y, 0.0f, 0.0f, initYaw, 0.0f};
    kf.init(initialX);
}

void ThreeDeadwheelChassisKFOdometry::update()
{
    assert(parallelOneCenterToWheelDistance + parallelTwoCenterToWheelDistance > 0);

    /* Process dead wheels */
    float mahonyOutput = 0.0f;
    if (!chassisYawObserver.getChassisWorldYaw(&mahonyOutput))
    {
        mahonyOutput = 0.0f;
        return;
    }

    Angle wrappedTheta = Angle(mahonyOutput);
    WrappedFloat deltaTheta = wrappedTheta - lastWrappedTheta;
    lastWrappedTheta = wrappedTheta;

    imuTheta += deltaTheta;

    // Get acceleration from IMU
    float Ax = imu.getAx();
    float Ay = imu.getAy();

    float imuOmega = imu.getGz();

    // Rotate acceleration to the world frame
    rotateVector(&Ax, &Ay, chassisYaw.getWrappedValue());

    /* Process dead wheels */

    float perpendicularRaw = deadwheelOdometry.getPerpendicularVelocity();
    float parallelOneRaw = deadwheelOdometry.getParallelMotorOneVelocity();
    float parallelTwoRaw =
        deadwheelOdometry.getParallelMotorTwoVelocity();  // EG@TODO: remove debug code

    // Compute odometry angular velocity
    float odoOmega = (parallelTwoRaw - parallelOneRaw) /
                     (parallelOneCenterToWheelDistance + parallelTwoCenterToWheelDistance);

    // Correct deadwheel velocities for rotational component
    float correctedParallelOne = parallelOneRaw + (odoOmega * parallelOneCenterToWheelDistance);
    float correctedParallelTwo = parallelTwoRaw - (odoOmega * parallelTwoCenterToWheelDistance);
    float correctedPerpendicular =
        perpendicularRaw - (odoOmega * perpendicularCenterToWheelDistance);

    // Average two parallel wheels to get velocity in odometry frame
    float Vx = (correctedParallelOne + correctedParallelTwo) / 2;
    float Vy = correctedPerpendicular;

    // Rotate velocity from odometry frame to robot frame
    rotateVector(&Vx, &Vy, odomFrameToRobotFrame);

    // Rotate velocity from robot frame to world frame
    rotateVector(&Vx, &Vy, chassisYaw.getWrappedValue());

    // Create the measurement vector
    float y[int(OdomInput::NUM_INPUTS)] =
        {Vx, Ax, Vy, Ay, imuTheta.getUnwrappedValue(), odoOmega, imuOmega};

    // Perform the Kalman filter update
    kf.performUpdate(y);
    updateChassisStateFromKF();
}

void ThreeDeadwheelChassisKFOdometry::updateChassisStateFromKF()
{
    auto stateVector = kf.getStateVectorAsMatrix();
    for (int i = 0; i < int(OdomState::NUM_STATES); i++)
    {
        x[i] = stateVector[i];
    }

    // update odometry velocity and orientation
    velocity.x = x[int(OdomState::VEL_X)];
    velocity.y = x[int(OdomState::VEL_Y)];

    chassisYaw = x[int(OdomState::POS_ANG)];
    location.setOrientation(x[int(OdomState::POS_ANG)]);
    location.setPosition(x[int(OdomState::POS_X)], x[int(OdomState::POS_Y)]);

    prevTime = tap::arch::clock::getTimeMicroseconds();
}

void ThreeDeadwheelChassisKFOdometry::overrideOdometryPosition(
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
