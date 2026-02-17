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

#include "aruwsrc/algorithms/eigen_extended_kalman_filter.hpp"

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
    : public aruwsrc::algorithms::EigenExtendedKalmanFilter<6, static_cast<uint16_t>(N * 6)>
{
public:
    using Base = aruwsrc::algorithms::EigenExtendedKalmanFilter<6, static_cast<uint16_t>(N * 6)>;
    static constexpr uint16_t kStateSize = 6;
    static constexpr uint16_t kMeasurementSize = static_cast<uint16_t>(N * 6);

    using StateVector = typename Base::StateVector;
    using InputVector = typename Base::InputVector;
    using StateMatrix = typename Base::StateMatrix;
    using InputMatrix = typename Base::InputMatrix;

    FusedImuEigenEkf(const StateMatrix& q, const InputMatrix& r, const StateMatrix& p0)
        : Base(
              stateTransitionFunction,
              observationFunction,
              stateJacobianFunction,
              nullptr,
              q,
              r,
              p0)
    {
    }

    int update(const InputVector& z) override
    {
        if (!this->initialized)
        {
            this->lastStatus = -1;
            return this->lastStatus;
        }

        for (uint16_t block = 0; block < N; block++)
        {
            const uint16_t rowBase = static_cast<uint16_t>(block * kStateSize);

            StateVector zBlock;
            for (uint16_t i = 0; i < kStateSize; i++)
            {
                zBlock(static_cast<int>(i), 0) = z(static_cast<int>(rowBase + i), 0);
            }

            const StateVector yBlock = zBlock - this->xHat;

            StateMatrix rBlock = StateMatrix::Zero();
            for (uint16_t r = 0; r < kStateSize; r++)
            {
                for (uint16_t c = 0; c < kStateSize; c++)
                {
                    rBlock(static_cast<int>(r), static_cast<int>(c)) =
                        this->Rdata[(rowBase + r) * kMeasurementSize + (rowBase + c)];
                }
            }

            const StateMatrix sBlock = this->P + rBlock;
            const StateMatrix kBlock = this->P * sBlock.inverse();

            this->xHat = this->xHat + kBlock * yBlock;
            this->P = (this->I - kBlock) * this->P;
        }

        this->syncStateArray();
        this->lastStatus = 0;
        return this->lastStatus;
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
        for (size_t imuIndex = 0; imuIndex < N; imuIndex++)
        {
            const size_t base = imuIndex * 6;
            predictedInput(static_cast<int>(base + 0), 0) = state(0, 0);
            predictedInput(static_cast<int>(base + 1), 0) = state(1, 0);
            predictedInput(static_cast<int>(base + 2), 0) = state(2, 0);
            predictedInput(static_cast<int>(base + 3), 0) = state(3, 0);
            predictedInput(static_cast<int>(base + 4), 0) = state(4, 0);
            predictedInput(static_cast<int>(base + 5), 0) = state(5, 0);
        }
    }

    static void stateJacobianFunction(
        const StateVector& state,
        StateMatrix& stateJacobian,
        float dt)
    {
        (void)state;
        (void)dt;
        stateJacobian.setIdentity();
    }
};

}  // namespace aruwsrc::communication::sensors::imu

#endif  // FUSED_IMU_EIGEN_EKF_HPP_
