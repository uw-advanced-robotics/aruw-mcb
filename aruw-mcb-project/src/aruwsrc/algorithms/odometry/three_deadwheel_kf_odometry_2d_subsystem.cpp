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
    const aruwsrc::algorithms::odometry::ThreeDeadwheelOdometryObserver &deadwheels,
#if defined(TARGET_SENTRY_ECLIPSE)
    tap::algorithms::odometry::ChassisWorldYawObserverInterface &yawObserver,
#else
    const aruwsrc::control::turret::TurretSubsystem &yawObserver,
#endif
    tap::communication::sensors::imu::ImuInterface &imu,
    float initialXPos,
    float initialYPos,
    float initialYaw,
    const float parallelOneCenterToWheelDistance,
    const float parallelTwoCenterToWheelDistance,
    const float perpendicularCenterToWheelDistance,
    const float odomFrameToRobotFrame)
    : Subsystem(&drivers),
      ThreeDeadwheelChassisKFOdometry(
          deadwheels,
#if defined(TARGET_SENTRY_ECLIPSE)
          yawObserver,
#else
          chassisYawObserver,
#endif
          imu,
          modm::Vector2f(initialXPos, initialYPos),
          initialYaw,
          parallelOneCenterToWheelDistance,
          parallelTwoCenterToWheelDistance,
          perpendicularCenterToWheelDistance,
          odomFrameToRobotFrame),
      chassisYawObserver(yawObserver)
{
}

void ThreeDeadwheelKFOdometry2DSubsystem::refresh() { update(); }

void ThreeDeadwheelKFOdometry2DSubsystem::overrideOdometryPosition(
    const float positionX,
    const float positionY)
{
    auto currKFState = this->kf.getStateVectorAsMatrix();

    float newState[int(ThreeDeadwheelChassisKFOdometry::OdomState::NUM_STATES)] = {
        positionX,
        currKFState[int(ThreeDeadwheelChassisKFOdometry::OdomState::VEL_X)],
        currKFState[int(ThreeDeadwheelChassisKFOdometry::OdomState::ACC_X)],
        positionY,
        currKFState[int(ThreeDeadwheelChassisKFOdometry::OdomState::VEL_Y)],
        currKFState[int(ThreeDeadwheelChassisKFOdometry::OdomState::ACC_Y)]};

    ThreeDeadwheelChassisKFOdometry::kf.init(newState);
}

void ThreeDeadwheelKFOdometry2DSubsystem::overrideOdometryOrientation(float deltaYaw)
{
    auto currKFState = this->kf.getStateVectorAsMatrix();

    float newState[int(ThreeDeadwheelChassisKFOdometry::OdomState::NUM_STATES)] = {
        currKFState[int(ThreeDeadwheelChassisKFOdometry::OdomState::POS_X)],
        currKFState[int(ThreeDeadwheelChassisKFOdometry::OdomState::VEL_X)],
        currKFState[int(ThreeDeadwheelChassisKFOdometry::OdomState::ACC_X)],
        currKFState[int(ThreeDeadwheelChassisKFOdometry::OdomState::POS_Y)],
        currKFState[int(ThreeDeadwheelChassisKFOdometry::OdomState::VEL_Y)],
        currKFState[int(ThreeDeadwheelChassisKFOdometry::OdomState::ACC_Y)]};

    tap::algorithms::rotateVector(
        &newState[int(ThreeDeadwheelChassisKFOdometry::OdomState::VEL_X)],
        &newState[int(ThreeDeadwheelChassisKFOdometry::OdomState::VEL_Y)],
        deltaYaw);

    tap::algorithms::rotateVector(
        &newState[int(ThreeDeadwheelChassisKFOdometry::OdomState::ACC_X)],
        &newState[int(ThreeDeadwheelChassisKFOdometry::OdomState::ACC_Y)],
        deltaYaw);

    ThreeDeadwheelChassisKFOdometry::kf.init(newState);
}

}  // namespace aruwsrc::algorithms::odometry
