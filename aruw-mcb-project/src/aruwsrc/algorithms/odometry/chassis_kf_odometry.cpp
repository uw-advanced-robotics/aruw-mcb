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

#include <array>
#include "chassis_kf_odometry.hpp"

namespace aruwsrc::algorithms::odometry
{
    static constexpr int STATES_SQUARED =
        static_cast<int>(ChassisKFOdometry::OdomState::STATES) * static_cast<int>(ChassisKFOdometry::OdomState::STATES);
    static constexpr int INPUTS_SQUARED =
        static_cast<int>(ChassisKFOdometry::OdomInput::INPUTS) * static_cast<int>(ChassisKFOdometry::OdomInput::INPUTS);
    static constexpr int INPUTS_MULT_STATES =
        static_cast<int>(ChassisKFOdometry::OdomInput::INPUTS) * static_cast<int>(ChassisKFOdometry::OdomState::STATES);

    static constexpr float DT = 0.002f;


    // clang-format off
    // static constexpr float KF_A[STATES_SQUARED] = {
    //     1, DT, 0.5 * DT * DT, 0, 0, 0,
    //     0,  1, DT, 0, 0, 0,
    //     0,  0, 1, 0, 0, 0,
    //     0, 0, 0, 1, DT, 0.5 * DT * DT,
    //     0, 0, 0, 0, 1, DT,
    //     0, 0, 0, 0, 0, 1,
    // };

    static constexpr float KF_A[STATES_SQUARED] = {
        1, DT, 0, 0, 0, 0,
        0, 0, DT, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, DT, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0,
    };

    static constexpr float KF_C[INPUTS_MULT_STATES] = {
        0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0,
    };
    static constexpr float KF_Q[STATES_SQUARED] = {
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1,
    };
    static constexpr float KF_R[INPUTS_SQUARED] = {
        0.1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 0.1, 0,
        0, 0, 0, 1,
    };
    static constexpr float KF_P[STATES_SQUARED] = {
        1E3, 0, 0, 0, 0, 0,
        0, 1E3, 0, 0, 0, 0,
        0, 0, 1E3, 0, 0, 0,
        0, 0, 0, 1E3, 0, 0,
        0, 0, 0, 0, 1E3, 0,
        0, 0, 0, 0, 0, 1E3,
    };
    // clang-format on


ChassisKFOdometry::ChassisKFOdometry(
    const tap::control::chassis::ChassisSubsystemInterface& chassisSubsystem,
    tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
    tap::communication::sensors::imu::ImuInterface& imu)
    : chassisSubsystem(chassisSubsystem),
      chassisYawObserver(chassisYawObserver),
      imu(imu),
      kf(KF_A, KF_C, KF_Q, KF_R, KF_P)
{
    float initialX[static_cast<int>(OdomState::STATES)] = {};
    kf.init(initialX);
}

void ChassisKFOdometry::update()
{
    float chassisYaw = 0;
    if (!chassisYawObserver.getChassisWorldYaw(&chassisYaw))
    {
        return;
    }

    // get chassis relative velocity
    modm::Matrix<float, 3, 1> chassisVelocity = chassisSubsystem.getActualVelocityChassisRelative();
    tap::control::chassis::ChassisSubsystemInterface::getVelocityWorldRelative(
        chassisVelocity,
        chassisYaw);

    // assume 0 velocity/acceleration in z direction
    float y[static_cast<int>(OdomInput::INPUTS)] = {};
    y[static_cast<int>(OdomInput::VEL_X)] = chassisVelocity[0][0];
    y[static_cast<int>(OdomInput::VEL_Y)] = chassisVelocity[1][0];
    y[static_cast<int>(OdomInput::ACC_X)] = imu.getAx();
    y[static_cast<int>(OdomInput::ACC_Y)] = imu.getAy();

    // acceleration in chassis frame, rotate to be in world frame
    tap::algorithms::rotateVector(
        &y[static_cast<int>(OdomInput::ACC_X)],
        &y[static_cast<int>(OdomInput::ACC_Y)],
        chassisYaw);

    // perform the update, new state matrix now available.
    kf.performUpdate(y);

    const auto& x = kf.getStateMatrix();

    // update odometry velocity and orientation
    velocity.x = x[static_cast<int>(OdomState::VEL_X)];
    velocity.y = x[static_cast<int>(OdomState::VEL_Y)];

    location.setOrientation(chassisYaw);
    location.setPosition(
        x[static_cast<int>(OdomState::POS_X)],
        x[static_cast<int>(OdomState::POS_Y)]);

#ifdef PLATFORM_HOSTED
    std::cout << x[static_cast<int>(OdomState::POS_X)] << ", "
              << x[static_cast<int>(OdomState::POS_Y)] << ", "
              << x[static_cast<int>(OdomState::VEL_X)] << ", "
              << x[static_cast<int>(OdomState::VEL_Y)] << std::endl;
#endif
}

}  // namespace aruwsrc::algorithms::odometry
