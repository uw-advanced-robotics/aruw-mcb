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

#ifndef OPTICAL_FLOW_KF_ODOMETRY_HPP_
#define OPTICAL_FLOW_KF_ODOMETRY_HPP_

#include "tap/algorithms/kalman_filter.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"

#include "aruwsrc/communication/serial/mtf_01.hpp"
#include "modm/math/geometry/vector.hpp"

using namespace tap::algorithms;
using namespace aruwsrc::communication::serial;
using namespace tap::communication::sensors::imu;

namespace aruwsrc::algorithms::odometry
{
/**
 * Class to calculate odometry using optical flow and IMU accelerometer data.
 */
class OpticalFlowKFOdometry
{
public:
    OpticalFlowKFOdometry(MTF01 &optical_flow, ImuInterface &imu);

    void update();

private:
    MTF01 &optical_flow;
    ImuInterface &imu;
    const float of_offset_degrees;

    static constexpr int STATES = 3;  // position, velocity, acceleration
    static constexpr int INPUTS = 2;  // optical flow, accelerometer

    tap::algorithms::KalmanFilter<STATES, INPUTS> xKF, yKF;

    /// Assumed time difference between calls to `update`, in seconds
    static constexpr float DT = 0.002f;

    // clang-format off
    // This is just physics
    static constexpr float KF_A[STATES * STATES] = {
        1.0f, DT,   0.5f * DT * DT,
        0.0f, 1.0f, DT,
        0.0f, 0.0f, 1.0f
    };

    // Observations directly correlate to the state
    static constexpr float KF_C[INPUTS * STATES] = {
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f
    };

    // These values are what Sparkfun use, we should trust physics
    static constexpr float KF_Q[STATES * STATES] = {
        1E8,  0.0f, 0.0f,
        0.0f, 1E6,  0.0f,
        0.0f, 0.0f, 1E4
    };

    // These should be std-dev ^ 2
    static constexpr float OPTICAL_FLOW_STD_DEV = 0.016f; // m/s
    static constexpr float ACCELEROMETER_STD_DEV = 0.1f; // m/s^2
    static constexpr float KF_R[INPUTS * INPUTS] = {
        OPTICAL_FLOW_STD_DEV * OPTICAL_FLOW_STD_DEV, 0.0f,
        0.0f, ACCELEROMETER_STD_DEV * ACCELEROMETER_STD_DEV
    };

    uint32_t prev_time;
};

}  // namespace aruwsrc::algorithms::odometry

#endif
