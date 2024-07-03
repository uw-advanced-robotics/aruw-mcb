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

#ifndef OPTICAL_FLOW_CF_ODOMETRY_HPP_
#define OPTICAL_FLOW_CF_ODOMETRY_HPP_

#include "tap/algorithms/transforms/vector.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"

#include "aruwsrc/communication/serial/mtf_01.hpp"

using namespace tap::algorithms::transforms;
using namespace aruwsrc::communication::serial;
using namespace tap::communication::sensors::imu;

namespace aruwsrc::algorithms::odometry
{
/**
 * Class to calculate odometry using optical flow and IMU accelerometer data.
 * Applies a filter to accel data and uses alpha as a balance in CF filter.
 */
class OpticalFlowCFOdometry
{
public:
    OpticalFlowCFOdometry(
        MTF01 &opticaFlow,
        ImuInterface &imu,
        float alpha,
        float offsetDeg);

    void update();

private:
    MTF01 &opticalFlow;
    ImuInterface &imu;
    float alpha;
    float offsetDeg;

    Vector currVelocity;
    Vector currPosition;
};

}  // namespace aruwsrc::algorithms::odometry

#endif
