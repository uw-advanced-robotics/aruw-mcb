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

#include "optical_flow_cf_odometry.hpp"

#include "tap/algorithms/math_user_utils.hpp"

using namespace tap::algorithms;
using namespace tap::algorithms::transforms;
using namespace aruwsrc::communication::serial;
using namespace tap::communication::sensors::imu;

namespace aruwsrc::algorithms::odometry
{
OpticalFlowCFOdometry::OpticalFlowCFOdometry(
    MTF01 &optical_flow,
    ImuInterface &imu,
    float alpha,
    float of_offset_degrees)
    : optical_flow(optical_flow),
      imu(imu),
      alpha(alpha),
      of_offset_degrees(of_offset_degrees),
      currVelocity(0, 0, 0),
      currPosition(0, 0, 0)
{
}

/**
 * Todos:
 * Filter the accel data
 */

void OpticalFlowCFOdometry::update()
{
    // Compute the accelerometers' velocity estimate by modifying current velocity estimate
    float imu_accel_x = imu.getAx();
    float imu_accel_y = imu.getAy();
    rotateVector(&imu_accel_x, &imu_accel_y, modm::toRadian(imu.getYaw()));
    Vector imu_accel = Vector(imu_accel_x, imu_accel_y, 0); // filterAndComputeAccelVector();

    Vector delta_vel = imu_accel * 0.002;
    Vector imu_vel_est = currVelocity + delta_vel;

    // Compute the optical flow velocity estimate
    float of_vel_x = optical_flow.getRelativeVelocity().x();
    float of_vel_y = optical_flow.getRelativeVelocity().y();
    rotateVector(&of_vel_x, &of_vel_y, modm::toRadian(of_offset_degrees + imu.getYaw()));
    Vector of_vel = Vector(of_vel_x, of_vel_y, 0);

    // Combine the two velocity estimates using alpha
    currVelocity = of_vel * alpha + imu_vel_est * (1 - alpha);

    // Compute the position estimate
    Vector delta_pos = currVelocity * 0.002;
    currPosition = currPosition + delta_pos;
}

}  // namespace aruwsrc::algorithms::odometry
