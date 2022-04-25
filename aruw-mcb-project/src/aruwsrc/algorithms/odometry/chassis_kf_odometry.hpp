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

#ifndef CHASSIS_KF_ODOMETRY_HPP_
#define CHASSIS_KF_ODOMETRY_HPP_

#include "tap/algorithms/kalman_filter.hpp"
#include "tap/algorithms/odometry/chassis_displacement_observer_interface.hpp"
#include "tap/algorithms/odometry/chassis_world_yaw_observer_interface.hpp"
#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"

#include "modm/math/geometry/location_2d.hpp"

namespace aruwsrc::algorithms::odometry
{
class ChassisKFOdometry : public tap::algorithms::odometry::Odometry2DInterface
{
public:
    ChassisKFOdometry(
        const tap::control::chassis::ChassisSubsystemInterface& chassisSubsystem,
        tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
        tap::communication::sensors::imu::ImuInterface& imu);

    inline modm::Location2D<float> getCurrentLocation2D() const final { return location; }

    inline modm::Vector2f getCurrentVelocity2D() const final { return velocity; }

    void update();

private:
    enum class OdomState
    {
        POS_X = 0,
        VEL_X,
        ACC_X,
        POS_Y,
        VEL_Y,
        ACC_Y,
        POS_Z,
        VEL_Z,
        ACC_Z,
        STATES,
    };

    enum class OdomInput
    {
        VEL_X = 0,
        ACC_X,
        VEL_Y,
        ACC_Y,
        VEL_Z,
        ACC_Z,
        INPUTS,
    };

    static constexpr int STATES_SQUARED =
        static_cast<int>(OdomState::STATES) * static_cast<int>(OdomState::STATES);
    static constexpr int INPUTS_SQUARED =
        static_cast<int>(OdomInput::INPUTS) * static_cast<int>(OdomInput::INPUTS);
    static constexpr int INPUTS_MULT_STATES =
        static_cast<int>(OdomInput::INPUTS) * static_cast<int>(OdomState::STATES);

    static constexpr float DT = 1;// 0.002f;

    // clang-format off
    static constexpr float KF_A[STATES_SQUARED] = {
        1, DT, 0.5 * DT * DT, 0, 0, 0, 0, 0, 0,
        0,  1, DT, 0, 0, 0, 0, 0, 0,
        0,  0, 1, 0, 0, 0, 0, 0, 0,

        0, 0, 0, 1, DT, 0.5 * DT * DT, 0, 0, 0,
        0, 0, 0, 0, 1, DT, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0,

        0, 0, 0, 0, 0, 0, 1, DT, 0.5 * DT * DT,
        0, 0, 0, 0, 0, 0, 0, 1, DT,
        0, 0, 0, 0, 0, 0, 0, 0, 1,
    };

    // static constexpr float KF_A[STATES_SQUARED] = {
    //     0, 0, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 0, 0, 0, 0, 0,
    // };

    static constexpr float KF_C[INPUTS_MULT_STATES] = {
        0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1,
    };
    static constexpr float KF_Q[STATES_SQUARED] = {
        1, 1, 1, 0, 0, 0, 0, 0, 0,
        1, 1, 1, 0, 0, 0, 0, 0, 0,
        1, 1, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 1, 1, 0, 0, 0,
        0, 0, 0, 1, 1, 1, 0, 0, 0,
        0, 0, 0, 1, 1, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 1, 1,
        0, 0, 0, 0, 0, 0, 1, 1, 1,
        0, 0, 0, 0, 0, 0, 1, 1, 1,
    };
    static constexpr float KF_R[INPUTS_SQUARED] = {
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1,
    };
    static constexpr float KF_P[STATES_SQUARED] = {
        1E3, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1E3, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1E3, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1E3, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1E3, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1E3, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1E3, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1E3, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1E3,
    };
    // clang-format on

    const tap::control::chassis::ChassisSubsystemInterface& chassisSubsystem;
    tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver;
    tap::communication::sensors::imu::ImuInterface& imu;

    tap::algorithms::
        KalmanFilter<static_cast<int>(OdomState::STATES), static_cast<int>(OdomInput::INPUTS)>
            kf;

    // Location in reference frame
    modm::Location2D<float> location;
    // Velocity in reference frame
    modm::Vector2f velocity;
};
}  // namespace aruwsrc::algorithms::odometry

#endif  // CHASSIS_KF_ODOMETRY_HPP_
