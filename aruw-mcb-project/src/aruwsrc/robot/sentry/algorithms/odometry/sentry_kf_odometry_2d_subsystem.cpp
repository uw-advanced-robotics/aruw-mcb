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

#include "sentry_kf_odometry_2d_subsystem.hpp"

#include "tap/drivers.hpp"

#include "sentry_chassis_world_yaw_observer.hpp"

namespace aruwsrc::sentry::algorithms::odometry
{
SentryKFOdometry2DSubsystem::SentryKFOdometry2DSubsystem(
    tap::Drivers &drivers,
    const aruwsrc::algorithms::odometry::TwoDeadwheelOdometryObserver &deadwheels,
    tap::algorithms::odometry::ChassisWorldYawObserverInterface &yawObserver,
    tap::communication::sensors::imu::ImuInterface &imu,
    float initialXPos,
    float initialYPos,
    const float centerToWheelDistance)
    : Subsystem(&drivers),
      TwoDeadwheelChassisKFOdometry(
          deadwheels,
          yawObserver,
          imu,
          modm::Vector2f(initialXPos, initialYPos),
          centerToWheelDistance,
          (1 / M_SQRT2),
          (1 / M_SQRT2))
{
}

void SentryKFOdometry2DSubsystem::refresh() { update(); }

void SentryKFOdometry2DSubsystem::overrideOdometryPosition(
    const float positionX,
    const float positionY)
{
    auto currKFState = this->kf.getStateVectorAsMatrix();

    float newState[int(TwoDeadwheelChassisKFOdometry::OdomState::NUM_STATES)] = {
        positionX,
        currKFState[int(TwoDeadwheelChassisKFOdometry::OdomState::VEL_X)],
        currKFState[int(TwoDeadwheelChassisKFOdometry::OdomState::ACC_X)],
        positionY,
        currKFState[int(TwoDeadwheelChassisKFOdometry::OdomState::VEL_Y)],
        currKFState[int(TwoDeadwheelChassisKFOdometry::OdomState::ACC_Y)]};

    TwoDeadwheelChassisKFOdometry::kf.init(newState);
}

void SentryKFOdometry2DSubsystem::overrideOdometryOrientation(float deltaYaw)
{
    auto currKFState = this->kf.getStateVectorAsMatrix();

    float newState[int(TwoDeadwheelChassisKFOdometry::OdomState::NUM_STATES)] = {
        currKFState[int(TwoDeadwheelChassisKFOdometry::OdomState::POS_X)],
        currKFState[int(TwoDeadwheelChassisKFOdometry::OdomState::VEL_X)],
        currKFState[int(TwoDeadwheelChassisKFOdometry::OdomState::ACC_X)],
        currKFState[int(TwoDeadwheelChassisKFOdometry::OdomState::POS_Y)],
        currKFState[int(TwoDeadwheelChassisKFOdometry::OdomState::VEL_Y)],
        currKFState[int(TwoDeadwheelChassisKFOdometry::OdomState::ACC_Y)]};

    tap::algorithms::rotateVector(
        &newState[int(TwoDeadwheelChassisKFOdometry::OdomState::VEL_X)],
        &newState[int(TwoDeadwheelChassisKFOdometry::OdomState::VEL_Y)],
        deltaYaw);

    tap::algorithms::rotateVector(
        &newState[int(TwoDeadwheelChassisKFOdometry::OdomState::ACC_X)],
        &newState[int(TwoDeadwheelChassisKFOdometry::OdomState::ACC_Y)],
        deltaYaw);

    TwoDeadwheelChassisKFOdometry::kf.init(newState);
}

}  // namespace aruwsrc::sentry::algorithms::odometry
