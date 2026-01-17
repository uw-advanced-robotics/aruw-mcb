/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include <cinttypes>
#include <functional>

#include "tap/algorithms/cmsis_mat.hpp"

#include "modm/architecture/interface/assert.h"
#include "modm/math/matrix.hpp"

namespace aruwsrc::algorithms
{
/**
 * Implementation of a multi-variable Extended Kalman Filter (EKF) that
 * utilizes arm's CMSIS matrix operations.
 *
 * The EKF handles nonlinear state transition and observation functions
 * by linearizing them around the current state estimate.
 *
 * @note Below, let \f$Y_{i - 1}\f$ be the set of all previous
 *      measurements, \f${y_1, y_2, ..., y_i\f$.
 */
template <uint16_t STATES, uint16_t INPUTS>
class ExtendedKalmanFilter
{
public:
    using StateVector = tap::algorithms::CMSISMat<STATES, 1>;
    using InputVector = tap::algorithms::CMSISMat<INPUTS, 1>;
    using StateMatrix = tap::algorithms::CMSISMat<STATES, STATES>;
    using InputMatrix = tap::algorithms::CMSISMat<INPUTS, INPUTS>;
    using ObservationMatrix = tap::algorithms::CMSISMat<INPUTS, STATES>;
    using KalmanGainMatrix = tap::algorithms::CMSISMat<STATES, INPUTS>;

    // Function type definitions for nonlinear functions
    using StateTransitionFunction = std::function<void(const StateVector &, StateVector &, float)>;
    using ObservationFunction = std::function<void(const StateVector &, InputVector &)>;
    using StateJacobianFunction = std::function<void(const StateVector &, StateMatrix &, float)>;
    using ObservationJacobianFunction =
        std::function<void(const StateVector &, ObservationMatrix &)>;

    /**
     * @param[in] f State transition function f(x, dt) -> x'
     * @param[in] h Observation function h(x) -> z
     * @param[in] F_jacobian Function to compute Jacobian of state transition function
     * @param[in] H_jacobian Function to compute Jacobian of observation function
     * @param[in] Q Process noise covariance.
     * @param[in] R Measurement error covariance.
     * @param[in] P0 Initial prediction error covariance estimate.
     */
    ExtendedKalmanFilter(
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

        // Compute Jacobian of state transition function at current state
        F_jacobian(xHat, F, dt);
        arm_mat_trans_f32(&F.matrix, &Ft.matrix);

        // Predict state using nonlinear state transition function
        StateVector xHat_prev = xHat;
        f(xHat_prev, xHat, dt);

        // Predict covariance
        P = F * P * Ft + Q;
    }

    int update(const InputVector &z)
    {
        if (!initialized)
        {
            return -1;
        }

        // Compute Jacobian of observation function at predicted state
        H_jacobian(xHat, H);
        if (arm_mat_trans_f32(&H.matrix, &Ht.matrix) != ARM_MATH_SUCCESS)
        {
            return 0;
        }

        // Predict measurement using nonlinear observation function
        h(xHat, z_pred);

        // Innovation (measurement residual)
        if (arm_mat_sub_f32(&z.matrix, &z_pred.matrix, &y.matrix) != ARM_MATH_SUCCESS)
        {
            return 1;
        }

        // Innovation covariance
        if (arm_mat_mult_f32(&H.matrix, &P.matrix, &HP.matrix) != ARM_MATH_SUCCESS)
        {
            return 2;
        }
        if (arm_mat_mult_f32(&HP.matrix, &Ht.matrix, &S.matrix) != ARM_MATH_SUCCESS)
        {
            return 3;
        }
        if (arm_mat_add_f32(&S.matrix, &R.matrix, &S.matrix) != ARM_MATH_SUCCESS)
        {
            return 4;
        }
        for (uint16_t i = 0; i < INPUTS; i++)
        {
            S.data[i * INPUTS + i] += 1.0e-6f;
        }

        // Kalman gain (guard against singular S)
        if (arm_mat_inverse_f32(&S.matrix, &S_inv.matrix) != ARM_MATH_SUCCESS)
        {
            return 5;
        }
        if (arm_mat_mult_f32(&P.matrix, &Ht.matrix, &K.matrix) != ARM_MATH_SUCCESS)
        {
            return 6;
        }
        if (arm_mat_mult_f32(&K.matrix, &S_inv.matrix, &K_tmp.matrix) != ARM_MATH_SUCCESS)
        {
            return 7;
        }
        K = K_tmp;

        // Update state estimate
        if (arm_mat_mult_f32(&K.matrix, &y.matrix, &K_y.matrix) != ARM_MATH_SUCCESS)
        {
            return 8;
        }
        if (arm_mat_add_f32(&xHat.matrix, &K_y.matrix, &xHat.matrix) != ARM_MATH_SUCCESS)
        {
            return 9;
        }

        // Update covariance estimate
        if (arm_mat_mult_f32(&K.matrix, &H.matrix, &KH.matrix) != ARM_MATH_SUCCESS)
        {
            return 10;
        }
        if (arm_mat_sub_f32(&I.matrix, &KH.matrix, &IKH.matrix) != ARM_MATH_SUCCESS)
        {
            return 11;
        }
        if (arm_mat_mult_f32(&IKH.matrix, &P.matrix, &P_new.matrix) != ARM_MATH_SUCCESS)
        {
            return 12;
        }
        P = P_new;
        return 13;
    }

    void performUpdate(const InputVector &z, float dt)
    {
        predict(dt);
        update(z);
    }

    const std::array<float, STATES> &getStateVectorAsMatrix() const { return xHat.data; }

    /**
     * @return Modifiable pointer to measurement covariance array so the covariance can be modified
     * at runtime if need be.
     */
    inline std::array<float, INPUTS * INPUTS> &getMeasurementCovariance() { return R.data; }

    /**
     * @return Modifiable pointer to process covariance array so the covariance can be modified
     * at runtime if need be.
     */
    inline std::array<float, STATES * STATES> &getProcessCovariance() { return Q.data; }

private:
    // Nonlinear functions
    StateTransitionFunction f;
    ObservationFunction h;
    StateJacobianFunction F_jacobian;
    ObservationJacobianFunction H_jacobian;

    /// System noise covariance
    StateMatrix Q;
    /// Measurement noise covariance
    InputMatrix R;

    /**
     * Predicted state matrix at the current time.
     */
    StateVector xHat;

    /**
     * Predicted error covariance.
     */
    StateMatrix P;

    /**
     * Initial error covariance.
     */
    StateMatrix P0;

    /**
     * Jacobian of state transition function (F matrix)
     */
    StateMatrix F;
    StateMatrix Ft;  // Transpose of F

    /**
     * Jacobian of observation function (H matrix)
     */
    ObservationMatrix H;
    tap::algorithms::CMSISMat<STATES, INPUTS> Ht;  // Transpose of H

    /**
     * Kalman filter gain matrix.
     */
    KalmanGainMatrix K;
    KalmanGainMatrix K_tmp;

    /**
     * Identity matrix created upon construction and stored to avoid
     * having to compute it each update step.
     */
    StateMatrix I;
    StateMatrix KH;
    StateMatrix IKH;
    StateMatrix P_new;
    InputMatrix HP;
    InputMatrix S;
    InputMatrix S_inv;
    InputVector z_pred;
    InputVector y;
    StateVector K_y;

    bool initialized = false;
};

}  // namespace aruwsrc::algorithms

#endif  // EXTENDED_KALMAN_FILTER_HPP_
