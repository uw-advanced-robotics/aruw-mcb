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
    tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
    tap::communication::sensors::imu::ImuInterface& imu,
    const modm::Vector2f initPos,
    const float initYaw,
    const float parallelOneCenterToWheelDistance,
    const float parallelTwoCenterToWheelDistance,
    const float perpendicularCenterToWheelDistance,
    const float odomFrameToRobotFrame)
    : kf_x(KF_A, KF_C, X_KF_Q, X_KF_R, X_KF_P0),
      kf_y(KF_A, KF_C, Y_KF_Q, Y_KF_R, Y_KF_P0),
      kf_ang(KF_A, KF_C, ANG_KF_Q, ANG_KF_R, ANG_KF_P0),
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

    kf_x.init({initPos.x, 0.0f});
    kf_y.init({initPos.y, 0.0f});
    kf_ang.init({initYaw, 0.0f});
}

void ThreeDeadwheelChassisKFOdometry::update()
{
    assert(parallelOneCenterToWheelDistance + parallelTwoCenterToWheelDistance > 0);

    /* Process IMU */
    /*float mahonyOutput = 0.0f;
    if (!chassisYawObserver.getChassisWorldYaw(&mahonyOutput))
    {
        mahonyOutput = 0.0f;
        return;
    }

    float wrappedTheta = Angle(mahonyOutput);
    WrappedFloat deltaTheta = wrappedTheta - lastWrappedTheta;
    lastWrappedTheta = wrappedTheta;

    imuTheta += deltaTheta;

    // Get acceleration from IMU
    float Ax = imu.getAx();
    float Ay = imu.getAy();

    float imuOmega = imu.getGz();

    // Rotate acceleration to the world frame
    rotateVector(&Ax, &Ay, chassisYaw.getWrappedValue());*/

    /* Process dead wheels */

    float perpendicularRaw = deadwheelOdometry.getPerpendicularVelocity();
    float parallelOneRaw = deadwheelOdometry.getParallelMotorOneVelocity();
    float parallelTwoRaw = deadwheelOdometry.getParallelMotorTwoVelocity();

    // Compute odometry angular velocity
    float odoOmega = (parallelTwoRaw - parallelOneRaw) /
                     (parallelOneCenterToWheelDistance + parallelTwoCenterToWheelDistance);

    // Correct deadwheel velocities for rotational component
    float correctedParallelOne = parallelOneRaw + (odoOmega * parallelOneCenterToWheelDistance);
    float correctedParallelTwo = parallelTwoRaw - (odoOmega * parallelTwoCenterToWheelDistance);
    float correctedPerpendicular = perpendicularRaw + (odoOmega * perpendicularCenterToWheelDistance);

    // Average two parallel wheels to get velocity in odometry frame
    float Vx = (correctedParallelOne + correctedParallelTwo) / 2;
    float Vy = correctedPerpendicular;

    // Rotate velocity from odometry frame to robot frame
    rotateVector(&Vx, &Vy, odomFrameToRobotFrame);

    // Rotate velocity from robot frame to world frame
    rotateVector(&Vx, &Vy, chassisYaw.getWrappedValue());

    // Create the measurement vector
    float x_measurement[int(XInput::NUM_INPUTS)] = {Vx};
    float y_measurement[int(YInput::NUM_INPUTS)] = {Vy};
    float ang_measurement[int(AngInput::NUM_INPUTS)] = {odoOmega};

    // Perform the Kalman filter update
    kf_x.performUpdate(x_measurement);
    kf_y.performUpdate(y_measurement);
    kf_ang.performUpdate(ang_measurement);

    updateChassisStateFromKF();
}

void ThreeDeadwheelChassisKFOdometry::updateChassisStateFromKF()
{
    auto xState = kf_x.getStateVectorAsMatrix();
    auto yState = kf_y.getStateVectorAsMatrix();
    auto angState = kf_ang.getStateVectorAsMatrix();

    velocity.x = xState[int(OdomStateX::VEL_X)];
    velocity.y = yState[int(OdomStateY::VEL_Y)];
    angular_velocity = angState[int(OdomStateAng::VEL_ANG)];

    float posAng = angState[int(OdomStateAng::POS_ANG)];
    chassisYaw = tap::algorithms::Angle(posAng);
    location.setOrientation(posAng);
    location.setPosition(xState[int(OdomStateX::POS_X)], yState[int(OdomStateY::POS_Y)]);

    prevTime = tap::arch::clock::getTimeMicroseconds();
}

void ThreeDeadwheelChassisKFOdometry::overrideOdometryPosition(
    const float positionX,
    const float positionY)
{
    auto x_currKFState = kf_x.getStateVectorAsMatrix();
    float x_newState[int(OdomStateX::NUM_STATES)] = {
        positionX,
        x_currKFState[int(OdomStateX::VEL_X)]};
    kf_x.init(x_newState);

    auto y_currKFState = kf_y.getStateVectorAsMatrix();
    float y_newState[int(OdomStateY::NUM_STATES)] = {
        positionY,
        y_currKFState[int(OdomStateY::VEL_Y)]};
    kf_y.init(y_newState);
}

void ThreeDeadwheelChassisKFOdometry::overrideOdometryOrientation(float deltaYaw)
{
    auto currKFState = this->kf_ang.getStateVectorAsMatrix();

    float newState[int(OdomStateAng::NUM_STATES)] = {
        currKFState[int(OdomStateAng::POS_ANG)] + deltaYaw,
        currKFState[int(OdomStateAng::VEL_ANG)]};
    kf_ang.init(newState);
}

}  // namespace aruwsrc::algorithms::odometry
