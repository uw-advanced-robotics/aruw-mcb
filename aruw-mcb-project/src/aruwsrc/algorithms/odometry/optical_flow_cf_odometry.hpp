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

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"
#include "modm/math/geometry/vector.hpp"

#include "aruwsrc/communication/serial/mtf_01.hpp"
#include "modm/math/filter/median.hpp"

using namespace tap::algorithms;
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
        MTF01 &optical_flow,
        ImuInterface &imu,
        float alpha,
        float of_offset_degrees);

    void update();

    void updateWithAccel();

private:
    MTF01 &optical_flow;
    ImuInterface &imu;
    float alpha;
    const float of_offset_degrees;

    modm::Vector2f currVelocity;
    modm::Vector2f currPosition;
    uint32_t prev_time;

    // Accelerometer filtration

    static constexpr int NUM_SAMPLES = 3;

    modm::filter::Median<float, NUM_SAMPLES> accel_x_filter;
    modm::filter::Median<float, NUM_SAMPLES> accel_y_filter;


    modm::Vector2f filterAndComputeAccelVector()
    {
        accel_x_filter.append(imu.getAx());
        accel_x_filter.update();
        float Ax = accel_x_filter.getValue();

        accel_y_filter.append(imu.getAy());
        accel_y_filter.update();
        float Ay = accel_y_filter.getValue();

        rotateVector(&Ax, &Ay, modm::toRadian(imu.getYaw()));
        return modm::Vector2f(Ax, Ay);
    }

    inline void updateDt_ms(uint32_t* dt){
        uint32_t curr_time = tap::arch::clock::getTimeMilliseconds();
        *dt = curr_time - prev_time;
        prev_time = curr_time;
    }
};

}  // namespace aruwsrc::algorithms::odometry

#endif
