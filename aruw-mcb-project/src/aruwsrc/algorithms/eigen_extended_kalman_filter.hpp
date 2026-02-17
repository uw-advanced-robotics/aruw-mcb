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

#ifndef ARUWSRC_EIGEN_EXTENDED_KALMAN_FILTER_HPP_
#define ARUWSRC_EIGEN_EXTENDED_KALMAN_FILTER_HPP_

#include <array>
#include <cinttypes>
#include <cstddef>
#include <cmath>
#include <cstdint>

#ifndef EIGEN_DONT_VECTORIZE
#define EIGEN_DONT_VECTORIZE
#endif
#ifndef EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#endif

#include <Eigen/Dense>

namespace aruwsrc::algorithms
{
/**
 * Implementation of a multi-variable Extended Kalman Filter (EKF) that
 * utilizes Eigen matrix operations.
 */
template <uint16_t STATES, uint16_t INPUTS>
class EigenExtendedKalmanFilter
{
public:
    using StateVector = Eigen::Matrix<float, STATES, 1, Eigen::DontAlign>;
    using InputVector = Eigen::Matrix<float, INPUTS, 1, Eigen::DontAlign>;
    using StateMatrix =
        Eigen::Matrix<float, STATES, STATES, Eigen::RowMajor | Eigen::DontAlign>;
    using InputMatrix =
        Eigen::Matrix<float, INPUTS, INPUTS, Eigen::RowMajor | Eigen::DontAlign>;
    using ObservationMatrix =
        Eigen::Matrix<float, INPUTS, STATES, Eigen::RowMajor | Eigen::DontAlign>;
    using KalmanGainMatrix =
        Eigen::Matrix<float, STATES, INPUTS, Eigen::RowMajor | Eigen::DontAlign>;

    using StateTransitionFunction = void (*)(const StateVector&, StateVector&, float);
    using ObservationFunction = void (*)(const StateVector&, InputVector&);
    using StateJacobianFunction = void (*)(const StateVector&, StateMatrix&, float);
    using ObservationJacobianFunction = void (*)(const StateVector&, ObservationMatrix&);

    EigenExtendedKalmanFilter(
        StateTransitionFunction f,
        ObservationFunction h,
        StateJacobianFunction F_jacobian,
        ObservationJacobianFunction H_jacobian,
        const float (&Q)[STATES * STATES],
        const float (&R)[INPUTS * INPUTS],
        const float (&P0)[STATES * STATES])
        : f(f),
          h(h),
          F_jacobian(F_jacobian),
          H_jacobian(H_jacobian),
          I(StateMatrix::Identity()),
          xHat(StateVector::Zero()),
          P(mapMatrix<StateMatrix>(P0)),
          P0(P),
          F(StateMatrix::Zero()),
          H(ObservationMatrix::Zero()),
          HPscratch(ObservationMatrix::Zero()),
          Sscratch(InputMatrix::Zero()),
          Kscratch(KalmanGainMatrix::Zero()),
          zPred(InputVector::Zero()),
          initialized(false)
    {
        copyArray(Q, Qdata);
        copyArray(R, Rdata);
        syncStateArray();
    }

    EigenExtendedKalmanFilter(
        StateTransitionFunction f,
        ObservationFunction h,
        StateJacobianFunction F_jacobian,
        ObservationJacobianFunction H_jacobian,
        const StateMatrix& Q,
        const InputMatrix& R,
        const StateMatrix& P0)
        : f(f),
          h(h),
          F_jacobian(F_jacobian),
          H_jacobian(H_jacobian),
          I(StateMatrix::Identity()),
          xHat(StateVector::Zero()),
          P(P0),
          P0(P0),
          F(StateMatrix::Zero()),
          H(ObservationMatrix::Zero()),
          HPscratch(ObservationMatrix::Zero()),
          Sscratch(InputMatrix::Zero()),
          Kscratch(KalmanGainMatrix::Zero()),
          zPred(InputVector::Zero()),
          initialized(false)
    {
        for (size_t i = 0; i < Qdata.size(); i++)
        {
            Qdata[i] = Q.data()[i];
        }
        for (size_t i = 0; i < Rdata.size(); i++)
        {
            Rdata[i] = R.data()[i];
        }
        syncStateArray();
    }

    void init(const float (&initialX)[STATES * 1])
    {
        xHat = mapVector<StateVector>(initialX);
        P = P0;
        initialized = true;
        debugStep = 0;
        lastStatus = 0;
        syncStateArray();
    }

    int predict(float dt)
    {
        debugStep = 1;
        debugFlags = 0;
        if (!initialized)
        {
            lastStatus = -1;
            return lastStatus;
        }

        debugStep = 2;
        F_jacobian(xHat, F, dt);
        debugStep = 3;
        StateVector xPrev = xHat;
        f(xPrev, xHat, dt);

        debugStep = 4;
        const StateMatrix Q = mapMatrix<StateMatrix>(Qdata.data());
        P = F * P * F.transpose() + Q;
        debugPNorm2 = P.squaredNorm();
        if (!std::isfinite(debugPNorm2)) debugFlags |= 0x01;
        debugStep = 5;
        lastStatus = 0;
        return lastStatus;
    }

    int update(const InputVector& z)
    {
        debugStep = 10;
        if (!initialized)
        {
            lastStatus = -1;
            return lastStatus;
        }

        debugStep = 11;
        debugStep = 111;
        if (H_jacobian)
        {
            H_jacobian(xHat, H);
        }
        else
        {
            if constexpr ((INPUTS % STATES) == 0)
            {
                // Special-case repeated-identity observation model:
                // z = [x; x; ...] + v. This avoids large H/S operations for multi-IMU fusion.
                constexpr uint16_t kBlocks = INPUTS / STATES;
                for (uint16_t block = 0; block < kBlocks; block++)
                {
                    debugStep = 113;
                    const uint16_t rowBase = block * STATES;

                    StateVector zBlock;
                    for (uint16_t i = 0; i < STATES; i++)
                    {
                        zBlock(static_cast<int>(i), 0) = z(static_cast<int>(rowBase + i), 0);
                    }

                    const StateVector yBlock = zBlock - xHat;
                    debugYNorm2 = yBlock.squaredNorm();

                    StateMatrix Rblock = StateMatrix::Zero();
                    for (uint16_t r = 0; r < STATES; r++)
                    {
                        for (uint16_t c = 0; c < STATES; c++)
                        {
                            Rblock(static_cast<int>(r), static_cast<int>(c)) =
                                Rdata[(rowBase + r) * INPUTS + (rowBase + c)];
                        }
                    }

                    const StateMatrix Sblock = P + Rblock;
                    debugSMinDiag = Sblock.diagonal().minCoeff();
                    debugSMaxDiag = Sblock.diagonal().maxCoeff();
                    debugRMinDiag = Rblock.diagonal().minCoeff();
                    debugRMaxDiag = Rblock.diagonal().maxCoeff();
                    if (!std::isfinite(debugSMinDiag) || !std::isfinite(debugSMaxDiag))
                    {
                        debugFlags |= 0x08;
                    }

                    const StateMatrix Kblock = P * Sblock.inverse();
                    debugKNorm2 = Kblock.squaredNorm();
                    if (!std::isfinite(debugKNorm2)) debugFlags |= 0x10;

                    xHat = xHat + Kblock * yBlock;
                    debugXNorm2 = xHat.squaredNorm();
                    if (!std::isfinite(debugXNorm2)) debugFlags |= 0x20;

                    P = (I - Kblock) * P;
                    debugPNorm2 = P.squaredNorm();
                    if (!std::isfinite(debugPNorm2)) debugFlags |= 0x40;
                }
                debugStep = 18;
                syncStateArray();
                lastStatus = 0;
                return lastStatus;
            }
            else
            {
                lastStatus = -2;
                return lastStatus;
            }
        }
        debugStep = 112;
        debugHNorm2 = H.squaredNorm();
        if (!std::isfinite(debugHNorm2)) debugFlags |= 0x02;
        debugStep = 12;
        h(xHat, zPred);

        debugStep = 13;
        const InputVector y = z - zPred;
        debugYNorm2 = y.squaredNorm();
        if (!std::isfinite(debugYNorm2)) debugFlags |= 0x04;
        debugStep = 14;
        debugStep = 141;
        HPscratch = H * P;
        debugStep = 142;
        Sscratch = HPscratch * H.transpose();
        debugStep = 143;
        const InputMatrix R = mapMatrix<InputMatrix>(Rdata.data());
        Sscratch = Sscratch + R;
        debugRMinDiag = R.diagonal().minCoeff();
        debugRMaxDiag = R.diagonal().maxCoeff();
        debugStep = 144;
        Sscratch.diagonal().array() += 1.0e-6f;
        debugSMinDiag = Sscratch.diagonal().minCoeff();
        debugSMaxDiag = Sscratch.diagonal().maxCoeff();
        if (!std::isfinite(debugSMinDiag) || !std::isfinite(debugSMaxDiag)) debugFlags |= 0x08;

        debugStep = 15;
        Kscratch = P * H.transpose() * Sscratch.inverse();
        debugKNorm2 = Kscratch.squaredNorm();
        if (!std::isfinite(debugKNorm2)) debugFlags |= 0x10;
        debugStep = 16;
        xHat = xHat + Kscratch * y;
        debugXNorm2 = xHat.squaredNorm();
        if (!std::isfinite(debugXNorm2)) debugFlags |= 0x20;
        debugStep = 17;
        P = (I - Kscratch * H) * P;
        debugPNorm2 = P.squaredNorm();
        if (!std::isfinite(debugPNorm2)) debugFlags |= 0x40;
        debugStep = 18;
        syncStateArray();
        lastStatus = 0;
        return lastStatus;
    }

    int performUpdate(const InputVector& z, float dt)
    {
        const int predictStatus = predict(dt);
        if (predictStatus != 0)
        {
            return predictStatus;
        }
        return update(z);
    }

    const std::array<float, STATES>& getStateVectorAsMatrix() const { return xHatArray; }

    inline std::array<float, INPUTS * INPUTS>& getMeasurementCovariance() { return Rdata; }
    inline std::array<float, STATES * STATES>& getProcessCovariance() { return Qdata; }
    inline int getLastStatus() const { return lastStatus; }
    inline uint8_t getDebugStep() const { return debugStep; }
    inline float getDebugYNorm2() const { return debugYNorm2; }
    inline float getDebugSMinDiag() const { return debugSMinDiag; }
    inline float getDebugSMaxDiag() const { return debugSMaxDiag; }
    inline float getDebugKNorm2() const { return debugKNorm2; }
    inline float getDebugHNorm2() const { return debugHNorm2; }
    inline float getDebugXNorm2() const { return debugXNorm2; }
    inline float getDebugPNorm2() const { return debugPNorm2; }
    inline float getDebugRMinDiag() const { return debugRMinDiag; }
    inline float getDebugRMaxDiag() const { return debugRMaxDiag; }
    inline uint8_t getDebugFlags() const { return debugFlags; }

private:
    template <typename MatrixT, typename DataT>
    static inline MatrixT mapMatrix(const DataT* data)
    {
        return Eigen::Map<const MatrixT>(data);
    }

    template <typename MatrixT, typename DataT>
    static inline MatrixT mapMatrix(const DataT (&data)[MatrixT::RowsAtCompileTime *
                                                       MatrixT::ColsAtCompileTime])
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
            xHatArray[i] = xHat(static_cast<int>(i), 0);
        }
    }

    StateTransitionFunction f;
    ObservationFunction h;
    StateJacobianFunction F_jacobian;
    ObservationJacobianFunction H_jacobian;

    std::array<float, STATES * STATES> Qdata{};
    std::array<float, INPUTS * INPUTS> Rdata{};

    const StateMatrix I;
    StateVector xHat;
    StateMatrix P;
    const StateMatrix P0;
    StateMatrix F;
    ObservationMatrix H;
    ObservationMatrix HPscratch;
    InputMatrix Sscratch;
    KalmanGainMatrix Kscratch;
    InputVector zPred;
    std::array<float, STATES> xHatArray{};
    bool initialized;
    int lastStatus = 0;
    volatile uint8_t debugStep = 0;
    volatile float debugYNorm2 = 0.0f;
    volatile float debugSMinDiag = 0.0f;
    volatile float debugSMaxDiag = 0.0f;
    volatile float debugKNorm2 = 0.0f;
    volatile float debugHNorm2 = 0.0f;
    volatile float debugXNorm2 = 0.0f;
    volatile float debugPNorm2 = 0.0f;
    volatile float debugRMinDiag = 0.0f;
    volatile float debugRMaxDiag = 0.0f;
    volatile uint8_t debugFlags = 0;
};
}  // namespace aruwsrc::algorithms

#endif  // ARUWSRC_EIGEN_EXTENDED_KALMAN_FILTER_HPP_
