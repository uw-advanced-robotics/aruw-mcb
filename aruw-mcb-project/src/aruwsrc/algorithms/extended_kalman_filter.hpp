/*
 * Copyright (c) 2020-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef EXTENDED_KALMAN_FILTER_HPP_
#define EXTENDED_KALMAN_FILTER_HPP_

#include <array>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <type_traits>

#include "tap/algorithms/cmsis_mat.hpp"

#include "modm/architecture/interface/assert.h"
#include "modm/math/matrix.hpp"

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
 * CMSIS backend EKF implementation.
 *
 * Uses arm CMSIS matrix operations and is preferred for smaller matrix sizes.
 */
template <uint16_t STATES, uint16_t INPUTS>
class ExtendedKalmanFilterCmsis
{
public:
    using StateVector = tap::algorithms::CMSISMat<STATES, 1>;
    using InputVector = tap::algorithms::CMSISMat<INPUTS, 1>;
    using StateMatrix = tap::algorithms::CMSISMat<STATES, STATES>;
    using InputMatrix = tap::algorithms::CMSISMat<INPUTS, INPUTS>;
    using ObservationMatrix = tap::algorithms::CMSISMat<INPUTS, STATES>;
    using KalmanGainMatrix = tap::algorithms::CMSISMat<STATES, INPUTS>;

    using StateTransitionFunction = void (*)(const StateVector&, StateVector&, float);
    using ObservationFunction = void (*)(const StateVector&, InputVector&);
    using StateJacobianFunction = void (*)(const StateVector&, StateMatrix&, float);
    using ObservationJacobianFunction = void (*)(const StateVector&, ObservationMatrix&);

    ExtendedKalmanFilterCmsis(
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
          Q(Q),
          R(R),
          xHat(),
          P(P0),
          P0(P0),
          F(),
          Ft(),
          H(),
          Ht(),
          K(),
          I()
    {
        I.constructIdentityMatrix();
    }

    ExtendedKalmanFilterCmsis(
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
          Q(Q),
          R(R),
          xHat(),
          P(P0),
          P0(P0),
          F(),
          Ft(),
          H(),
          Ht(),
          K(),
          I()
    {
        I.constructIdentityMatrix();
    }

    void init(const float (&initialX)[STATES * 1])
    {
        xHat.copyData(initialX);
        P.data = P0.data;
        initialized = true;
    }

    void predict(float dt)
    {
        if (!initialized)
        {
            return;
        }

        F_jacobian(xHat, F, dt);
        arm_mat_trans_f32(&F.matrix, &Ft.matrix);

        StateVector xHat_prev = xHat;
        f(xHat_prev, xHat, dt);
        (void)arm_mat_mult_f32(&F.matrix, &P.matrix, &FP.matrix);
        (void)arm_mat_mult_f32(&FP.matrix, &Ft.matrix, &P_pred.matrix);
        (void)arm_mat_add_f32(&P_pred.matrix, &Q.matrix, &P.matrix);
    }

    int update(const InputVector& z) { return static_cast<int>(updateImpl(z, -1)); }

    int updateWrapped(const InputVector& z, uint16_t wrappedIndex)
    {
        return static_cast<int>(
            updateImpl(z, wrappedIndex < INPUTS ? static_cast<int>(wrappedIndex) : -1));
    }

private:
    enum CMSISErrorCodes : int
    {
        NOT_INITIALIZED = -1,
        TRANSPOSE_H_FAILED = 1,
        CALCULATE_Y_FAILED = 2,
        H_TIMES_P_FAILED = 3,
        H_TIMES_Ht_FAILED = 4,
        ADDING_R_TO_S_FAILED = 5,
        S_INVERSION_FAILED = 6,
        P_TIMES_Ht_FAILED = 7,
        K_TIMES_S_INV_FAILED = 8,
        K_TIMES_Y_FAILED = 9,
        ADDING_KY_TO_XHAT_FAILED = 10,
        K_TIMES_H_FAILED = 11,
        IKH_SUBTRACTION_FAILED = 12,
        IKH_TIMES_P_FAILED = 13,
        SUCCESS = 0
    };
    CMSISErrorCodes updateImpl(const InputVector& z, int wrappedResidualIndex)
    {
        if (!initialized)
        {
            return NOT_INITIALIZED;
        }

        H_jacobian(xHat, H);
        // Ht
        if (arm_mat_trans_f32(&H.matrix, &Ht.matrix) != ARM_MATH_SUCCESS)
        {
            return TRANSPOSE_H_FAILED;
        }

        h(xHat, z_pred);
        // y = z - z_pred
        if (arm_mat_sub_f32(&z.matrix, &z_pred.matrix, &y.matrix) != ARM_MATH_SUCCESS)
        {
            return CALCULATE_Y_FAILED;
        }
        if (wrappedResidualIndex >= 0)
        {
            float& wrappedResidual = y.data[static_cast<size_t>(wrappedResidualIndex)];
            wrappedResidual = std::atan2(std::sin(wrappedResidual), std::cos(wrappedResidual));
        }

        // S = H * P * Ht + R

        // Hp = H * P
        if (arm_mat_mult_f32(&H.matrix, &P.matrix, &HP.matrix) != ARM_MATH_SUCCESS)
        {
            return H_TIMES_P_FAILED;
        }
        // S = Hp * Ht
        if (arm_mat_mult_f32(&HP.matrix, &Ht.matrix, &S.matrix) != ARM_MATH_SUCCESS)
        {
            return H_TIMES_Ht_FAILED;
        }
        // S = S + R -> S = H * P * Ht + R
        if (arm_mat_add_f32(&S.matrix, &R.matrix, &S.matrix) != ARM_MATH_SUCCESS)
        {
            return ADDING_R_TO_S_FAILED;
        }
        for (uint16_t i = 0; i < INPUTS; i++)
        {
            S.data[i * INPUTS + i] += 1.0e-6f;
        }

        // S^-1
        if (arm_mat_inverse_f32(&S.matrix, &S_inv.matrix) != ARM_MATH_SUCCESS)
        {
            return S_INVERSION_FAILED;
        }
        // K = P * Ht
        if (arm_mat_mult_f32(&P.matrix, &Ht.matrix, &K.matrix) != ARM_MATH_SUCCESS)
        {
            return P_TIMES_Ht_FAILED;
        }
        // K = K * S^-1
        if (arm_mat_mult_f32(&K.matrix, &S_inv.matrix, &K_tmp.matrix) != ARM_MATH_SUCCESS)
        {
            return K_TIMES_S_INV_FAILED;
        }
        K = K_tmp;

        // xHat = xHat + K * y
        if (arm_mat_mult_f32(&K.matrix, &y.matrix, &K_y.matrix) != ARM_MATH_SUCCESS)
        {
            return K_TIMES_Y_FAILED;
        }
        if (arm_mat_add_f32(&xHat.matrix, &K_y.matrix, &xHat.matrix) != ARM_MATH_SUCCESS)
        {
            return ADDING_KY_TO_XHAT_FAILED;
        }

        // P = (I - K * H) * P
        // KH = K * H
        if (arm_mat_mult_f32(&K.matrix, &H.matrix, &KH.matrix) != ARM_MATH_SUCCESS)
        {
            return K_TIMES_H_FAILED;
        }
        // IKH = I - KH
        if (arm_mat_sub_f32(&I.matrix, &KH.matrix, &IKH.matrix) != ARM_MATH_SUCCESS)
        {
            return IKH_SUBTRACTION_FAILED;
        }
        // P = IKH * P
        if (arm_mat_mult_f32(&IKH.matrix, &P.matrix, &P_new.matrix) != ARM_MATH_SUCCESS)
        {
            return IKH_TIMES_P_FAILED;
        }
        P = P_new;
        return SUCCESS;
    }

public:
    void performUpdate(const InputVector& z, float dt)
    {
        predict(dt);
        update(z);
    }

    const std::array<float, STATES>& getStateVectorAsMatrix() const { return xHat.data; }
    inline std::array<float, STATES>& getMutableStateVector() { return xHat.data; }
    inline std::array<float, STATES * STATES>& getMutableStateCovariance() { return P.data; }
    inline std::array<float, INPUTS * INPUTS>& getMeasurementCovariance() { return R.data; }
    inline std::array<float, STATES * STATES>& getProcessCovariance() { return Q.data; }

private:
    StateTransitionFunction f;
    ObservationFunction h;
    StateJacobianFunction F_jacobian;
    ObservationJacobianFunction H_jacobian;

    StateMatrix Q;
    InputMatrix R;

    StateVector xHat;
    StateMatrix P;
    StateMatrix P0;

    StateMatrix F;
    StateMatrix Ft;

    ObservationMatrix H;
    tap::algorithms::CMSISMat<STATES, INPUTS> Ht;

    KalmanGainMatrix K;
    KalmanGainMatrix K_tmp;

    StateMatrix I;
    StateMatrix KH;
    StateMatrix IKH;
    StateMatrix P_new;
    StateMatrix FP;
    StateMatrix P_pred;
    ObservationMatrix HP;
    InputMatrix S;
    InputMatrix S_inv;
    InputVector z_pred;
    InputVector y;
    StateVector K_y;

    bool initialized = false;
};

/**
 * Eigen backend EKF implementation.
 *
 * Uses fixed-size Eigen matrices and is needed for >10x10
 */
template <uint16_t STATES, uint16_t INPUTS>
class ExtendedKalmanFilterEigen
{
public:
    using StateVector = Eigen::Matrix<float, STATES, 1, Eigen::DontAlign>;
    using InputVector = Eigen::Matrix<float, INPUTS, 1, Eigen::DontAlign>;
    using StateMatrix = Eigen::Matrix<float, STATES, STATES, Eigen::RowMajor | Eigen::DontAlign>;
    using InputMatrix = Eigen::Matrix<float, INPUTS, INPUTS, Eigen::RowMajor | Eigen::DontAlign>;
    using ObservationMatrix =
        Eigen::Matrix<float, INPUTS, STATES, Eigen::RowMajor | Eigen::DontAlign>;
    using KalmanGainMatrix =
        Eigen::Matrix<float, STATES, INPUTS, Eigen::RowMajor | Eigen::DontAlign>;

    using StateTransitionFunction = void (*)(const StateVector&, StateVector&, float);
    using ObservationFunction = void (*)(const StateVector&, InputVector&);
    using StateJacobianFunction = void (*)(const StateVector&, StateMatrix&, float);
    using ObservationJacobianFunction = void (*)(const StateVector&, ObservationMatrix&);

    enum EigenErrorCodes
    {
        NOT_INITIALIZED = -1,
        NO_H_JACOBIAN = -2,
        SUCCESS = 0
    };

    ExtendedKalmanFilterEigen(
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

    ExtendedKalmanFilterEigen(
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

    virtual ~ExtendedKalmanFilterEigen() = default;

    void init(const float (&initialX)[STATES * 1])
    {
        xHat = mapVector<StateVector>(initialX);
        P = P0;
        initialized = true;
        lastStatus = SUCCESS;
        syncStateArray();
    }

    int predict(float dt)
    {
        if (!initialized)
        {
            lastStatus = NOT_INITIALIZED;
            return static_cast<int>(lastStatus);
        }

        F_jacobian(xHat, F, dt);
        StateVector xPrev = xHat;
        f(xPrev, xHat, dt);

        const StateMatrix Q = mapMatrix<StateMatrix>(Qdata.data());
        P = F * P * F.transpose() + Q;
        lastStatus = SUCCESS;
        return static_cast<int>(lastStatus);
    }

    virtual int update(const InputVector& z)
    {
        if (!initialized)
        {
            lastStatus = NOT_INITIALIZED;
            return static_cast<int>(lastStatus);
        }

        if (!H_jacobian)
        {
            lastStatus = NO_H_JACOBIAN;
            return static_cast<int>(lastStatus);
        }

        H_jacobian(xHat, H);
        h(xHat, zPred);

        const InputVector y = z - zPred;
        HPscratch = H * P;
        Sscratch = HPscratch * H.transpose();
        const InputMatrix R = mapMatrix<InputMatrix>(Rdata.data());
        Sscratch = Sscratch + R;
        Sscratch.diagonal().array() += 1.0e-6f;

        Kscratch = P * H.transpose() * Sscratch.inverse();
        xHat = xHat + Kscratch * y;
        P = (I - Kscratch * H) * P;
        syncStateArray();
        lastStatus = SUCCESS;
        return static_cast<int>(lastStatus);
    }

    int performUpdate(const InputVector& z, float dt)
    {
        const int predictStatus = predict(dt);
        if (static_cast<EigenErrorCodes>(predictStatus) != SUCCESS)
        {
            return predictStatus;
        }
        return update(z);
    }

    const std::array<float, STATES>& getStateVectorAsMatrix() const { return xHatArray; }

    inline std::array<float, INPUTS * INPUTS>& getMeasurementCovariance() { return Rdata; }
    inline std::array<float, STATES * STATES>& getProcessCovariance() { return Qdata; }
    inline EigenErrorCodes getLastStatus() const { return lastStatus; }

protected:
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
    EigenErrorCodes lastStatus = NOT_INITIALIZED;
};

/**
 * Selects the EKF backend by compile-time matrix sizes.
 *
 * CMSIS is selected for small systems where it is typically faster/lighter,
 * and Eigen is selected for larger systems.
 */
template <uint16_t STATES, uint16_t INPUTS>
inline constexpr bool USE_CMSIS_EKF_BACKEND = (STATES <= 10 && INPUTS <= 10);

/**
 * Unified EKF type alias.
 *
 * Uses CMSIS when `USE_CMSIS_EKF_BACKEND` is true, otherwise Eigen.
 */
template <uint16_t STATES, uint16_t INPUTS>
using ExtendedKalmanFilter = std::conditional_t<
    USE_CMSIS_EKF_BACKEND<STATES, INPUTS>,
    ExtendedKalmanFilterCmsis<STATES, INPUTS>,
    ExtendedKalmanFilterEigen<STATES, INPUTS>>;

}  // namespace aruwsrc::algorithms

#endif  // EXTENDED_KALMAN_FILTER_HPP_
