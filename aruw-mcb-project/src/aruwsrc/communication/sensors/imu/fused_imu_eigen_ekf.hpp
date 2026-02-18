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
 * Backend adapter specialized by the concrete EKF type selected in
 * `ExtendedKalmanFilter`.
 */
template <typename FilterT>
struct FusedImuEkfBackendAdapter;

template <uint16_t States, uint16_t Inputs>
struct FusedImuEkfBackendAdapter<aruwsrc::algorithms::ExtendedKalmanFilterCmsis<States, Inputs>>
{
    using FilterType = aruwsrc::algorithms::ExtendedKalmanFilterCmsis<States, Inputs>;
    static constexpr bool usesCmsisBackend = true;

    static inline int predict(FilterType& filter, float dt)
    {
        filter.predict(dt);
        return 0;
    }

    template <typename MatrixT>
    static inline void setIdentity(MatrixT& matrix)
    {
        matrix.constructIdentityMatrix();
    }

    template <typename MatrixT>
    static inline void setZero(MatrixT& matrix)
    {
        for (auto& value : matrix.data)
        {
            value = 0.0f;
        }
    }

    template <typename MatrixT>
    static inline void setMatrixElement(
        MatrixT& matrix,
        size_t row,
        size_t col,
        float value,
        size_t columnCount)
    {
        matrix.data[row * columnCount + col] = value;
    }

    template <typename VectorT>
    static inline void setVectorElement(VectorT& vector, size_t row, float value)
    {
        vector.data[row] = value;
    }

    template <typename MatrixT>
    static inline float getMatrixElement(
        const MatrixT& matrix,
        size_t row,
        size_t col,
        size_t columnCount)
    {
        return matrix.data[row * columnCount + col];
    }
};

template <uint16_t States, uint16_t Inputs>
struct FusedImuEkfBackendAdapter<aruwsrc::algorithms::ExtendedKalmanFilterEigen<States, Inputs>>
{
    using FilterType = aruwsrc::algorithms::ExtendedKalmanFilterEigen<States, Inputs>;
    static constexpr bool usesCmsisBackend = false;

    static inline int predict(FilterType& filter, float dt) { return filter.predict(dt); }

    template <typename MatrixT>
    static inline void setIdentity(MatrixT& matrix)
    {
        matrix.setIdentity();
    }

    template <typename MatrixT>
    static inline void setZero(MatrixT& matrix)
    {
        matrix.setZero();
    }

    template <typename MatrixT>
    static inline void setMatrixElement(
        MatrixT& matrix,
        size_t row,
        size_t col,
        float value,
        size_t /* columnCount */)
    {
        matrix(static_cast<int>(row), static_cast<int>(col)) = value;
    }

    template <typename VectorT>
    static inline void setVectorElement(VectorT& vector, size_t row, float value)
    {
        vector(static_cast<int>(row), 0) = value;
    }

    template <typename MatrixT>
    static inline float getMatrixElement(
        const MatrixT& matrix,
        size_t row,
        size_t col,
        size_t /* columnCount */)
    {
        return matrix(static_cast<int>(row), static_cast<int>(col));
    }
};

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
    static constexpr uint16_t stateSize = 6;

    using StateVector = typename Base::StateVector;
    using InputVector = typename Base::InputVector;
    using StateMatrix = typename Base::StateMatrix;
    using InputMatrix = typename Base::InputMatrix;
    using ObservationMatrix = typename Base::ObservationMatrix;
    using BackendAdapter = FusedImuEkfBackendAdapter<Base>;

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
    int predict(float dt) { return BackendAdapter::predict(static_cast<Base&>(*this), dt); }

    template <typename MatrixT>
    static inline void zeroMatrix(MatrixT& matrix)
    {
        BackendAdapter::setZero(matrix);
    }

    template <typename MatrixT>
    static inline void setMatrixElement(MatrixT& matrix, size_t row, size_t col, float value)
    {
        BackendAdapter::setMatrixElement(matrix, row, col, value, stateSize);
    }

    template <typename VectorT>
    static inline void setVectorElement(VectorT& vector, size_t row, float value)
    {
        BackendAdapter::setVectorElement(vector, row, value);
    }

    int updateSingleImu(uint16_t imuIndex, const InputVector& zBlock)
    {
        if (imuIndex >= static_cast<uint16_t>(N))
        {
            return -3;
        }

        auto& measurementCovariance = this->getMeasurementCovariance();
        for (int r = 0; r < static_cast<int>(stateSize); r++)
        {
            for (int c = 0; c < static_cast<int>(stateSize); c++)
            {
                measurementCovariance[static_cast<size_t>(r) * stateSize + static_cast<size_t>(c)] =
                    BackendAdapter::getMatrixElement(
                        measurementCovarianceBlocks[imuIndex],
                        static_cast<size_t>(r),
                        static_cast<size_t>(c),
                        stateSize);
            }
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
        BackendAdapter::setIdentity(stateJacobian);
    }

    static void observationJacobianFunction(
        const StateVector& state,
        ObservationMatrix& observationJacobian)
    {
        (void)state;
        BackendAdapter::setIdentity(observationJacobian);
    }

    std::array<InputMatrix, N> measurementCovarianceBlocks{};
};

}  // namespace aruwsrc::communication::sensors::imu

#endif  // FUSED_IMU_EIGEN_EKF_HPP_
