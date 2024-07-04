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

#include "optical_flow_kf_odometry.hpp"

using namespace tap::algorithms;
using namespace aruwsrc::communication::serial;
using namespace tap::communication::sensors::imu;

namespace aruwsrc::algorithms::odometry
{

OpticalFlowKFOdometry::OpticalFlowKFOdometry(
    MTF01 &optical_flow,
    ImuInterface &imu,
    const float of_offset_degrees)
    : optical_flow(optical_flow),
      imu(imu),
      of_offset_degrees(of_offset_degrees),
      xKF(KF_A, KF_C, KF_Q, KF_R, KF_P0),
      yKF(KF_A, KF_C, KF_Q, KF_R, KF_P0)
{
}

void OpticalFlowKFOdometry::update()
{
    // Get optical flow data
    auto of_data = optical_flow.getRelativeVelocity();
    rotateVector(&of_data.x, &of_data.y, modm::toRadian(imu.getYaw() + of_offset_degrees));

    // Get accelerometer data
    float acc_x = imu.getAx();
    float acc_y = imu.getAy();
    rotateVector(&acc_x, &acc_y, modm::toRadian(imu.getYaw()));

    // Update Kalman filters
    float xY[INPUTS] = {of_data.x, acc_x};
    float yY[INPUTS] = {of_data.y, acc_y};

    xKF.performUpdate(xY);
    yKF.performUpdate(yY);

    estimated_position.x = xKF.getStateVectorAsMatrix()[0];
    estimated_position.y = yKF.getStateVectorAsMatrix()[0];

    estimated_velocity.x = xKF.getStateVectorAsMatrix()[1];
    estimated_velocity.y = yKF.getStateVectorAsMatrix()[1];

    estimated_acceleration.x = xKF.getStateVectorAsMatrix()[2];
    estimated_acceleration.y = yKF.getStateVectorAsMatrix()[2];
}

void OpticalFlowKFOdometry::reset()
{
    float initialPos[STATES] = {0.0f, 0.0f, 0.0f};
    xKF.init(initialPos);
    yKF.init(initialPos);
}

}  // namespace aruwsrc::algorithms::odometry
