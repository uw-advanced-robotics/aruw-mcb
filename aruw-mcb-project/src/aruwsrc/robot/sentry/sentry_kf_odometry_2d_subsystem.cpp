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

#include "sentry_kf_odometry_2d_subsystem.hpp"
#include "sentry_chassis_world_yaw_observer.hpp"

#include "tap/drivers.hpp"

namespace aruwsrc::sentry
{
SentryKFOdometry2DSubsystem::SentryKFOdometry2DSubsystem(
    tap::Drivers &drivers,
    tap::control::chassis::ChassisSubsystemInterface &chassis,
    SentryChassisWorldYawObserver &yawObserver,
    tap::communication::sensors::imu::ImuInterface &imu,
    modm::Location2D<float> imuToChassisCenter,
    float initialXPos,
    float initialYPos)
    : Subsystem(&drivers),
      ChassisKFOdometry(chassis, yawObserver, imu, imuToChassisCenter, initialXPos, initialYPos)
{
}

void SentryKFOdometry2DSubsystem::refresh() { update(); }

void SentryKFOdometry2DSubsystem::overrideOdometry(const modm::Vector2f& newPos, const float& deltaYaw) {

  // auto currKFState = ChassisKFOdometry::kf.getStateVectorAsMatrix();
  auto currKFState = this->kf.getStateVectorAsMatrix();

  float newState[int(ChassisKFOdometry::OdomState::NUM_STATES)] = {
    newPos.x,
    currKFState[int(ChassisKFOdometry::OdomState::VEL_X)], // TOOD: rotate these
    currKFState[int(ChassisKFOdometry::OdomState::ACC_X)],
    newPos.y,
    currKFState[int(ChassisKFOdometry::OdomState::VEL_Y)],
    currKFState[int(ChassisKFOdometry::OdomState::ACC_Y)]
  };

  tap::algorithms::rotateVector(
      &newState[int(ChassisKFOdometry::OdomInput::VEL_X)],
      &newState[int(ChassisKFOdometry::OdomInput::VEL_Y)],
      deltaYaw);

  tap::algorithms::rotateVector(
      &newState[int(ChassisKFOdometry::OdomInput::ACC_X)],
      &newState[int(ChassisKFOdometry::OdomInput::ACC_Y)],
      deltaYaw);

 ChassisKFOdometry::kf.init(newState);
}


}  // namespace aruwsrc::sentry
