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

#include <cmath>

#include "three_deadwheel_chassis_kf_odometry.hpp"

namespace aruwsrc::algorithms::odometry
{
using namespace tap::algorithms;

ThreeDeadwheelChassisKFOdometry::ThreeDeadwheelChassisKFOdometry(
    std::array<Deadwheel*, 3> deadwheels,
    tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
    tap::communication::sensors::imu::ImuInterface& imu,
    const modm::Vector2f initPos,
    const float initYaw)
    : kf_x(KF_A, KF_C, X_KF_Q, X_KF_R, X_KF_P0),
      kf_y(KF_A, KF_C, Y_KF_Q, Y_KF_R, Y_KF_P0),
      kf_ang(KF_A, KF_C, ANG_KF_Q, ANG_KF_R, ANG_KF_P0),
      deadwheels(deadwheels),
      chassisYawObserver(chassisYawObserver),
      imu(imu),
      initPos(initPos),
      initYaw(initYaw),
      chassisYaw(initYaw)
{
    tap::algorithms::CMSISMat<3, 3> A;
    for (int i = 0; i < 3; i++)
    {
        const auto* deadwheel = deadwheels[i];
        const float theta = deadwheel->getWheelTheta();

        A[i * 3 + 0] = std::cos(theta);
        A[i * 3 + 1] = std::sin(theta);
        A[i * 3 + 2] = deadwheel->getX() * std::sin(theta)
                     - deadwheel->getY() * std::cos(theta);
    }

    AI = A.inverse();
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
    // float mahonyOutput = 0.0f;
    // if (!chassisYawObserver.getChassisWorldYaw(&mahonyOutput))
    // {
    //     return;
    // }

    // wrappedTheta = Angle(mahonyOutput);
    // WrappedFloat deltaTheta = wrappedTheta - lastWrappedTheta;
    // lastWrappedTheta = wrappedTheta;
    // imuTheta += deltaTheta;

    tap::algorithms::CMSISMat<3, 1> wheelVelocity;
    for (int i = 0; i < 3; i++)
    {
        wheelVelocity[i] = deadwheels[i]->getVelocity();
    }
    tap::algorithms::CMSISMat<3, 1> robotVelocity = AI * wheelVelocity;

    rotateVector(&robotVelocity[0], &robotVelocity[1], chassisYaw.getWrappedValue());

    float x_measurement[int(XInput::NUM_INPUTS)] = {robotVelocity[0]};
    float y_measurement[int(YInput::NUM_INPUTS)] = {robotVelocity[1]};
    float ang_measurement[int(AngInput::NUM_INPUTS)] = {robotVelocity[2]};

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
    auto xCurrState = kf_x.getStateVectorAsMatrix();
    float xNewState[int(OdomStateX::NUM_STATES)] = {positionX, xCurrState[int(OdomStateX::VEL_X)]};
    kf_x.init(xNewState);

    auto yCurrState = kf_y.getStateVectorAsMatrix();
    float yNewState[int(OdomStateY::NUM_STATES)] = {positionY, yCurrState[int(OdomStateY::VEL_Y)]};
    kf_y.init(yNewState);
}

void ThreeDeadwheelChassisKFOdometry::overrideOdometryOrientation(float deltaYaw)
{
    auto currKFState = kf_ang.getStateVectorAsMatrix();

    float newState[int(OdomStateAng::NUM_STATES)] = {
        currKFState[int(OdomStateAng::POS_ANG)] + deltaYaw,
        currKFState[int(OdomStateAng::VEL_ANG)]};
    kf_ang.init(newState);
}

}  // namespace aruwsrc::algorithms::odometry
