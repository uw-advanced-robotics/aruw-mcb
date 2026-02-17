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

#ifndef EIGEN_KALMAN_FILTER_HPP_
#define EIGEN_KALMAN_FILTER_HPP_

#include <array>
#include <cinttypes>
#include <cstddef>

#include <Eigen/Dense>

namespace aruwsrc::algorithms
{
/**
 * Implementation of a multi-variable linear kalman filter that
 * utilizes Eigen matrix operations.
 *
 * @note Below, let \f$Y_{i - 1}\f$ be the set of all previous
 *      measurements, \f${y_1, y_2, ..., y_i\f$.
 */
template <uint16_t STATES, uint16_t INPUTS>
class EigenKalmanFilter
{
public:
    using MatrixA = Eigen::Matrix<float, STATES, STATES, Eigen::RowMajor>;
    using MatrixC = Eigen::Matrix<float, INPUTS, STATES, Eigen::RowMajor>;
    using MatrixQ = Eigen::Matrix<float, STATES, STATES, Eigen::RowMajor>;
    using MatrixR = Eigen::Matrix<float, INPUTS, INPUTS, Eigen::RowMajor>;
    using MatrixP = Eigen::Matrix<float, STATES, STATES, Eigen::RowMajor>;
    using MatrixK = Eigen::Matrix<float, STATES, INPUTS, Eigen::RowMajor>;
    using MatrixI = Eigen::Matrix<float, STATES, STATES, Eigen::RowMajor>;
    using VectorX = Eigen::Matrix<float, STATES, 1>;
    using VectorY = Eigen::Matrix<float, INPUTS, 1>;

    /**
     * @param[in] A State transition matrix (also called F).
     * @param[in] C Observation matrix (also called H).
     * @param[in] Q Process noise covariance.
     * @param[in] R Measurement error covariance.
     * @param[in] P0 Initial prediction error covariance estimate.
     */
    EigenKalmanFilter(
        const float (&A)[STATES * STATES],
        const float (&C)[INPUTS * STATES],
        const float (&Q)[STATES * STATES],
        const float (&R)[INPUTS * INPUTS],
        const float (&P0)[STATES * STATES])
        : A(mapMatrix<MatrixA>(A)),
          At(A.transpose()),
          C(mapMatrix<MatrixC>(C)),
          Ct(C.transpose()),
          Q(mapMatrix<MatrixQ>(Q)),
          P0(mapMatrix<MatrixP>(P0)),
          I(MatrixI::Identity()),
          xHat(VectorX::Zero()),
          P(P0)
    {
        copyArray(R, Rdata);
        syncStateArray();
    }

    EigenKalmanFilter(
        const MatrixA& A,
        const MatrixC& C,
        const MatrixQ& Q,
        const MatrixR& R,
        const MatrixP& P0)
        : A(A),
          At(A.transpose()),
          C(C),
          Ct(C.transpose()),
          Q(Q),
          P0(P0),
          I(MatrixI::Identity()),
          xHat(VectorX::Zero()),
          P(P0)
    {
        for (size_t i = 0; i < Rdata.size(); i++)
        {
            Rdata[i] = R.data()[i];
        }
        syncStateArray();
    }

    void init(const float (&initialX)[STATES * 1])
    {
        xHat = mapVector<VectorX>(initialX);
        P = P0;
        initialized = true;
        syncStateArray();
    }

    void performUpdate(const VectorY& y)
    {
        if (!initialized)
        {
            return;
        }

        xHat = A * xHat;
        P = A * P * At + Q;

        const MatrixR R = mapMatrix<MatrixR>(Rdata.data());
        const MatrixK K = P * Ct * (C * P * Ct + R).inverse();
        xHat = xHat + K * (y - C * xHat);
        P = (I - K * C) * P;

        syncStateArray();
    }

    const std::array<float, STATES>& getStateVectorAsMatrix() const { return xHatArray; }

    /**
     * @return Modifiable pointer to measurement covariance array so the covariance can be modified
     * at runtime if need be.
     */
    inline std::array<float, INPUTS * INPUTS>& getMeasurementCovariance() { return Rdata; }

private:
    template <typename MatrixT, typename DataT>
    static inline MatrixT mapMatrix(const DataT* data)
    {
        return Eigen::Map<const MatrixT>(data);
    }

    template <typename MatrixT, typename DataT>
    static inline MatrixT mapMatrix(
        const DataT (&data)[MatrixT::RowsAtCompileTime * MatrixT::ColsAtCompileTime])
    {
        return Eigen::Map<const MatrixT>(data);
    }

    template <typename VectorT, typename DataT>
    static inline VectorT mapVector(const DataT (&data)[VectorT::RowsAtCompileTime * 1])
    {
        return Eigen::Map<const VectorT>(data);
    }

    template <typename DataT, size_t N>
    static inline void copyArray(const DataT (&src)[N], std::array<float, N>& dst)
    {
        for (size_t i = 0; i < N; i++)
        {
            dst[i] = static_cast<float>(src[i]);
        }
    }

    inline void syncStateArray()
    {
        for (size_t i = 0; i < STATES; i++)
        {
            xHatArray[i] = xHat(static_cast<int>(i));
        }
    }

    /// State transition matrix.
    const MatrixA A;
    /// Transpose of A.
    const MatrixA At;
    /// Observation matrix.
    const MatrixC C;
    /// Transpose of C.
    const MatrixC Ct;
    /// System noise covariance.
    const MatrixQ Q;
    /// Initial error covariance.
    const MatrixP P0;
    /// Identity matrix.
    const MatrixI I;
    /// Predicted state matrix at the current time.
    VectorX xHat;
    /// Predicted error covariance.
    MatrixP P;
    /// Measurement noise covariance.
    std::array<float, INPUTS * INPUTS> Rdata{};
    /// Cached state vector to match tap kalman filter API.
    std::array<float, STATES> xHatArray{};

    bool initialized = false;
};
}  // namespace aruwsrc::algorithms

#endif  // EIGEN_KALMAN_FILTER_HPP_
