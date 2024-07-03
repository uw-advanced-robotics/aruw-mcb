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

using namespace tap::algorithms;
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
      of_offset_degrees(of_offset_degrees)
{
}

void OpticalFlowCFOdometry::updateWithAccel()
{
    uint32_t dt;
    updateDt_ms(&dt);

    modm::Vector2f accel = filterAndComputeAccelVector();
    // Compute the accelerometers' velocity estimate by modifying current velocity estimate

    modm::Vector2f delta_vel = accel * (dt / 1000.0f);
    modm::Vector2f imu_vel_est = currVelocity + delta_vel;

    // Compute the optical flow velocity estimate
    modm::Vector2f of_vel = optical_flow.getRelativeVelocity();
    rotateVector(&of_vel.x, &of_vel.y, modm::toRadian(of_offset_degrees + imu.getYaw()));

    // Combine the two velocity estimates using alpha
    currVelocity = of_vel * alpha + imu_vel_est * (1 - alpha);

    // Compute the position estimate
    modm::Vector2f delta_pos = currVelocity * (dt / 1000.0f);
    currPosition = currPosition + delta_pos;
}

modm::Vector2f of_vel, delta_pos;
void OpticalFlowCFOdometry::update()
{
    uint32_t dt;
    updateDt_ms(&dt);

    // Compute the optical flow velocity estimate
    of_vel = optical_flow.getRelativeVelocity();
    rotateVector(&of_vel.x, &of_vel.y, modm::toRadian(of_offset_degrees + imu.getYaw()));

    // Compute the position estimate
    delta_pos = of_vel * (dt / 1000.0f);
    currPosition = currPosition + delta_pos;
}

}  // namespace aruwsrc::algorithms::odometry
