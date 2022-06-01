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

#ifndef SENTINEL_CHASSIS_KF_ODOMETRY_HPP_
#define SENTINEL_CHASSIS_KF_ODOMETRY_HPP_

#include "tap/algorithms/kalman_filter.hpp"
#include "tap/algorithms/odometry/chassis_displacement_observer_interface.hpp"
#include "tap/algorithms/odometry/chassis_world_yaw_observer_interface.hpp"
#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"

#include "modm/math/geometry/location_2d.hpp"
#include "modm/math/interpolation/linear.hpp"

namespace aruwsrc::algorithms::odometry
{
/**
 * An odometry interface that uses a kalman filter to measure odometry. This class is designed
 * specifically for the sentinel.
 *
 * @note Assumes the world frame has an origin of (0, 0) wherever the robot was booted from.
 */
class SentinelChassisKFOdometry : public tap::algorithms::odometry::Odometry2DInterface
{
public:
    SentinelChassisKFOdometry(
        const tap::control::chassis::ChassisSubsystemInterface& chassisSubsystem,
        tap::communication::sensors::imu::ImuInterface& imu);

    inline modm::Location2D<float> getCurrentLocation2D() const final { return location; }

    inline modm::Vector2f getCurrentVelocity2D() const final { return velocity; }

    inline uint32_t getLastComputedOdometryTime() const final { return prevTime; }

    void update();

private:
    enum class OdomState
    {
        POS_Y = 0,
        VEL_Y,
        ACC_Y,
        NUM_STATES,
    };

    enum class OdomInput
    {
        POS_Y = 0,
        ACC_Y,
        NUM_INPUTS,
    };

    static constexpr int STATES_SQUARED =
        static_cast<int>(OdomState::NUM_STATES) * static_cast<int>(OdomState::NUM_STATES);
    static constexpr int INPUTS_SQUARED =
        static_cast<int>(OdomInput::NUM_INPUTS) * static_cast<int>(OdomInput::NUM_INPUTS);
    static constexpr int INPUTS_MULT_STATES =
        static_cast<int>(OdomInput::NUM_INPUTS) * static_cast<int>(OdomState::NUM_STATES);

    /// Assumed time difference between calls to `update`, in seconds
    static constexpr float DT = 0.002f;

    // clang-format off
    static constexpr float KF_A[STATES_SQUARED] = {
        1, DT, 0.5 * DT * DT,
        0, 1 , DT           ,
        0, 0 , 1            ,
    };
    static constexpr float KF_C[INPUTS_MULT_STATES] = {
        1, 0, 0,
        0, 0, 1,
    };
    static constexpr float KF_Q[STATES_SQUARED] = {
        1E0, 0  , 0  ,
        0  , 1E0, 0  ,
        0  , 0  , 1E0,
    }; // TODO: Tune for sentinel 2022
    static constexpr float KF_R[INPUTS_SQUARED] = {
        1.0, 0  ,
        0  , 1.2,
    }; // TODO: Tune for sentinel 2022
    static constexpr float KF_P0[STATES_SQUARED] = {
        1E3, 0  , 0  ,
        0  , 1E3, 0  ,
        0  , 0  , 1E3,
    }; // TODO: Tune for sentinel 2022
    // clang-format on

    /// Max chassis acceleration magnitude measured on the sentinel when at 120W power mode, in
    /// m/s^2
    static constexpr float MAX_ACCELERATION = 8.0f;  // TODO: Tune for sentinel 2022

    // TODO: Tune for sentinel 2022
    static constexpr modm::Pair<float, float> CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT[] =
        {
            {0, 10E-2},
            {MAX_ACCELERATION, 10E2},
        };

    static constexpr float CHASSIS_WHEEL_ACCELERATION_LOW_PASS_ALPHA =
        0.01f;  // TODO: Tune for sentinel 2022

    const tap::control::chassis::ChassisSubsystemInterface& chassisSubsystem;
    tap::communication::sensors::imu::ImuInterface& imu;

    tap::algorithms::KalmanFilter<
        static_cast<int>(OdomState::NUM_STATES),
        static_cast<int>(OdomInput::NUM_INPUTS)>
        kf;

    // Location in reference frame
    modm::Location2D<float> location;
    // Velocity in reference frame
    modm::Vector2f velocity;

    /// Chassis-measured change in velocity since the last time `update` was called, in the chassis
    /// frame
    float chassisMeasuredDeltaVelocity;

    modm::interpolation::Linear<modm::Pair<float, float>>
        chassisAccelerationToMeasurementCovarianceInterpolator;

    /// Previous time `update` was called, in microseconds
    uint32_t prevTime = 0;
    float prevChassisVelocity;

    void updateChassisStateFromKF();

    void updateMeasurementCovariance(const float& chassisVelocity);
};
}  // namespace aruwsrc::algorithms::odometry

#endif  // CHASSIS_KF_ODOMETRY_HPP_
