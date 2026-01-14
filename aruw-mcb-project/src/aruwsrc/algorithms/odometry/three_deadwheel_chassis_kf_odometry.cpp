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
ThreeDeadwheelChassisKFOdometry::ThreeDeadwheelChassisKFOdometry(
    const aruwsrc::algorithms::odometry::ThreeDeadwheelOdometryObserver& deadwheelOdometry,
#if defined(TARGET_SENTRY_ECLIPSE)
    tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
#else
    aruwsrc::algorithms::odometry::OttoChassisWorldYawObserver& chassisYawObserver,
#endif
    tap::communication::sensors::imu::ImuInterface& imu,
    const modm::Vector2f initPos,
    const float parallelOneCenterToWheelDistance,
    const float parallelTwoCenterToWheelDistance,
    const float perpendicularCenterToWheelDistance,
    const float parallelWheelOneChassisForwardRelativeAngleRadians,
    const float parallelWheelTwoChassisForwardRelativeAngleRadians,
    const float perpendicularWheelChassisForwardRelativeAngleRadians)
    : kf(KF_A, KF_C, KF_Q, KF_R, KF_P0),
      deadwheelOdometry(deadwheelOdometry),
      chassisYawObserver(chassisYawObserver),
      imu(imu),
      initPos(initPos),
      parallelOneCenterToWheelDistance(parallelOneCenterToWheelDistance),
      parallelTwoCenterToWheelDistance(parallelTwoCenterToWheelDistance),
      perpendicularCenterToWheelDistance(perpendicularCenterToWheelDistance),
      parallelWheelOneChassisForwardRelativeAngleRadians(
          parallelWheelOneChassisForwardRelativeAngleRadians),
      parallelWheelTwoChassisForwardRelativeAngleRadians(
          parallelWheelTwoChassisForwardRelativeAngleRadians),
      perpendicularWheelChassisForwardRelativeAngleRadians(
          perpendicularWheelChassisForwardRelativeAngleRadians)
{
    reset();
}

void ThreeDeadwheelChassisKFOdometry::reset()
{
    float initialX[int(OdomState::NUM_STATES)] =
        {initPos.x, 0.0f, 0.0f, initPos.y, 0.0f, 0.0f, 0.0f, 0.0f};
    kf.init(initialX);
}

// may or may not work
float ThreeDeadwheelChassisKFOdometry::applyIirFilter(
    float input,
    float* state,
    const float* a,
    const float* b,
    int order)
{
    for (int i = order - 1; i > 0; i--)
    {
        state[i] = state[i - 1];
    }

    float output = b[0] * input;
    for (int i = 1; i < order; i++)
    {
        output += b[i] * state[i];
        output -= a[i] * state[i - 1];
    }

    state[0] = input;
    return output;
}

void ThreeDeadwheelChassisKFOdometry::update()
{
    float chassisYaw = location.getOrientation();
    /* Process dead wheels */

    float yawIMU;
    if (!chassisYawObserver.getChassisWorldYaw(&yawIMU))
    {
        yawIMU = 0;
        return;
    }

    // Get acceleration from IMU
    float Ax = imu.getAx();
    float Ay = imu.getAy();

    float Wimu = imu.getGz();

    // Rotate acceleration to the world frame
    tap::algorithms::rotateVector(&Ax, &Ay, chassisYaw);

    /* Process dead wheels */

    perpendicularRaw = deadwheelOdometry.getPerpendicularVelocity();
    parallelOneRaw = deadwheelOdometry.getParallelMotorOneVelocity();
    parallelTwoRaw = deadwheelOdometry.getParallelMotorTwoVelocity();

    // Compute odometry angular velocity
    float Wodo = (parallelTwoRaw - parallelOneRaw) /
                 (parallelOneCenterToWheelDistance + parallelTwoCenterToWheelDistance);

    // Correct deadwheel velocities for rotational component
    float correctedParallelOne = parallelOneRaw + (Wodo * parallelOneCenterToWheelDistance);
    float correctedParallelTwo = parallelTwoRaw - (Wodo * parallelTwoCenterToWheelDistance);
    float correctedPerpendicular = perpendicularRaw - (Wodo * perpendicularCenterToWheelDistance);

    filteredParallelOne =
        applyIirFilter(correctedParallelOne, parallelOneFilterState, IIR_A, IIR_B, FILTER_ORDER);

    filteredParallelTwo =
        applyIirFilter(correctedParallelTwo, parallelTwoFilterState, IIR_A, IIR_B, FILTER_ORDER);

    filteredPerpendicular = applyIirFilter(
        correctedPerpendicular,
        perpendicularFilterState,
        IIR_A,
        IIR_B,
        FILTER_ORDER);

    // Correct for deadwheel orientation and average the two parallel wheels
    float Vx =
        ((filteredParallelOne * std::sin(parallelWheelOneChassisForwardRelativeAngleRadians) +
          (filteredParallelTwo * std::sin(parallelWheelTwoChassisForwardRelativeAngleRadians))) /
             2 +
         filteredPerpendicular * std::cos(perpendicularWheelChassisForwardRelativeAngleRadians));
    float Vy =
        ((filteredParallelOne * std::cos(parallelWheelOneChassisForwardRelativeAngleRadians) +
          (filteredParallelTwo * std::cos(parallelWheelTwoChassisForwardRelativeAngleRadians))) /
             2 +
         filteredPerpendicular * std::sin(perpendicularWheelChassisForwardRelativeAngleRadians));

    tap::algorithms::rotateVector(&Vx, &Vy, chassisYaw);

    // Create the measurement vector
    float y[int(OdomInput::NUM_INPUTS)] = {Vx, Ax, Vy, Ay, chassisYaw, Wodo, Wimu};

    // Perform the Kalman filter update
    kf.performUpdate(y);
    updateChassisStateFromKF();
}

void ThreeDeadwheelChassisKFOdometry::updateChassisStateFromKF()
{
    const auto& x = kf.getStateVectorAsMatrix();

    // update odometry velocity and orientation
    velocity.x = x[int(OdomState::VEL_X)];
    velocity.y = x[int(OdomState::VEL_Y)];

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
