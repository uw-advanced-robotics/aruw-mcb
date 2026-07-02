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

#include "three_deadwheel_kf_odometry_2d_subsystem.hpp"

#include "tap/drivers.hpp"

namespace aruwsrc::algorithms::odometry
{
ThreeDeadwheelKFOdometry2DSubsystem::ThreeDeadwheelKFOdometry2DSubsystem(
    tap::Drivers &drivers,
    std::array<aruwsrc::algorithms::odometry::ThreeDeadwheelOdometryObserver*, 3> deadwheels,
    tap::algorithms::odometry::ChassisWorldYawObserverInterface &yawObserver,
    tap::communication::sensors::imu::ImuInterface &imu,
    float initialXPos,
    float initialYPos,
    float initialYaw)
    : Subsystem(&drivers),
      ThreeDeadwheelChassisKFOdometry(
          deadwheels,
          yawObserver,
          imu,
          modm::Vector2f(initialXPos, initialYPos),
          initialYaw)
{
}

void ThreeDeadwheelKFOdometry2DSubsystem::refresh() { update(); }

void ThreeDeadwheelKFOdometry2DSubsystem::overrideOdometryPosition(
    const float positionX,
    const float positionY)
{
    auto xState = kf_x.getStateVectorAsMatrix();
    float x_newState[int(OdomStateX::NUM_STATES)] = {positionX, xState[int(OdomStateX::VEL_X)]};
    kf_x.init(x_newState);

    auto yState = kf_y.getStateVectorAsMatrix();
    float y_newState[int(OdomStateY::NUM_STATES)] = {positionY, yState[int(OdomStateY::VEL_Y)]};
    kf_y.init(y_newState);
}

void ThreeDeadwheelKFOdometry2DSubsystem::overrideOdometryOrientation(float deltaYaw)
{
    auto angState = kf_ang.getStateVectorAsMatrix();
    float ang_newState[int(OdomStateAng::NUM_STATES)] = {
        angState[int(OdomStateAng::POS_ANG)] + deltaYaw,
        angState[int(OdomStateAng::VEL_ANG)]};
    kf_ang.init(ang_newState);
}

}  // namespace aruwsrc::algorithms::odometry
