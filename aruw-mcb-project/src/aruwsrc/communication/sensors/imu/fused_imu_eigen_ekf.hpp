/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef FUSED_IMU_EIGEN_EKF_HPP_
#define FUSED_IMU_EIGEN_EKF_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

#include "aruwsrc/algorithms/extended_kalman_filter.hpp"

namespace aruwsrc::communication::sensors::imu
{
/**
 * EKF extension for fused IMU state/measurement model.
 *
 * State x: [ax, ay, az, gx, gy, gz]^T
 * Measurement z: N stacked copies of state (one per IMU)
 */
template <size_t N>
class FusedImuEigenEkf
    : public aruwsrc::algorithms::ExtendedKalmanFilter<6, 6>
{
public:
    using Base = aruwsrc::algorithms::ExtendedKalmanFilter<6, 6>;
    static constexpr uint16_t kStateSize = 6;

    using StateVector = typename Base::StateVector;
    using InputVector = typename Base::InputVector;
    using StateMatrix = typename Base::StateMatrix;
    using InputMatrix = typename Base::InputMatrix;
    using ObservationMatrix = typename Base::ObservationMatrix;

    FusedImuEigenEkf(
        const StateMatrix& q,
        const std::array<InputMatrix, N>& rBlocks,
        const StateMatrix& p0)
        : Base(
              stateTransitionFunction,
              observationFunction,
              stateJacobianFunction,
              observationJacobianFunction,
              q,
              rBlocks[0],
              p0),
          measurementCovarianceBlocks(rBlocks)
    {
    }

    int update(const InputVector& z) { return updateSingleImu(0, z); }

    int predict(float dt)
    {
        Base::predict(dt);
        return 0;
    }

    int updateSingleImu(uint16_t imuIndex, const InputVector& zBlock)
    {
        if (imuIndex >= static_cast<uint16_t>(N))
        {
            return -3;
        }

        auto& measurementCovariance = this->getMeasurementCovariance();
        for (size_t i = 0; i < kStateSize * kStateSize; i++)
        {
            measurementCovariance[i] = measurementCovarianceBlocks[imuIndex].data[i];
        }

        return Base::update(zBlock);
    }

    inline std::array<InputMatrix, N>& getMeasurementCovarianceBlocks()
    {
        return measurementCovarianceBlocks;
    }

private:
    static void stateTransitionFunction(
        const StateVector& state,
        StateVector& predictedState,
        float dt)
    {
        (void)dt;
        predictedState = state;
    }

    static void observationFunction(const StateVector& state, InputVector& predictedInput)
    {
        predictedInput = state;
    }

    static void stateJacobianFunction(
        const StateVector& state,
        StateMatrix& stateJacobian,
        float dt)
    {
        (void)state;
        (void)dt;
        stateJacobian.constructIdentityMatrix();
    }

    static void observationJacobianFunction(
        const StateVector& state,
        ObservationMatrix& observationJacobian)
    {
        (void)state;
        observationJacobian.constructIdentityMatrix();
    }

    std::array<InputMatrix, N> measurementCovarianceBlocks{};
};

}  // namespace aruwsrc::communication::sensors::imu

#endif  // FUSED_IMU_EIGEN_EKF_HPP_
