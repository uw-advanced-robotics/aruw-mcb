/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TAPROOT_KALMAN_FILTER_HPP_
#define TAPROOT_KALMAN_FILTER_HPP_

#include <cinttypes>

#include "modm/math/matrix.hpp"
#include "modm/architecture/interface/assert.h"

#include "matrix_utils.hpp"

namespace tap::algorithms
{
/**
 * Implementation of a multi-variable linear kalman filter that
 * utilizes arm's CMSIS matrix operations.
 *
 * @note Below, let \f$Y_{i - 1}\f$ be the set of all previous
 *      measurements, \f${y_1, y_2, ..., y_i\f$.
 */
template <uint16_t STATES, uint16_t INPUTS>
class KalmanFilter
{
public:
    /**
     * @param[in] A State transition matrix (also called F).
     * @param[in] C Observation matrix (also called H).
     * @param[in] Q Process noise covariance.
     * @param[in] R Measurement error covariance.
     * @param[in] P Initial prediction error covariance estimate.
     */
    KalmanFilter(
        const float (&A)[STATES * STATES],
        const float (&C)[INPUTS * STATES],
        const float (&Q)[STATES * STATES],
        const float (&R)[INPUTS * INPUTS],
        const float (&P)[STATES * STATES])
        : A(A),
          At(),
          C(C),
          Ct(),
          Q(Q),
          R(R),
          xHat(),
          P(P),
          P0(P),
          K(),
          I(),
          CPCtR(),
          CPCtRInverse()
    {
        arm_mat_init_f32(&CPCtRARM, INPUTS, INPUTS, CPCtR.element);
        arm_mat_init_f32(&CPCtRInverseARM, INPUTS, INPUTS, CPCtRInverse.element);

        At = this->A.asTransposed();
        Ct = this->C.asTransposed();
        I = modm::Matrix<float, STATES, STATES>::identityMatrix();
    }

    void init(const float (&initialX)[STATES * 1])
    {
        xHat = initialX;
        P = P0;

        initialized = true;
    }

#ifdef PLATFORM_HOSTED
template<int ROWS, int COLS>
    inline void print(const modm::Matrix<float, ROWS, COLS> &m) const
    {
        for (size_t i = 0; i < ROWS; i++)
        {
            for (size_t j = 0; j < COLS; j++)
            {
                std::cout << m.element[i * COLS + j] << (j != COLS - 1 ? " " : "");
            }
            if (i != ROWS - 1)
            {
                std::cout << '\n';
            }
        }
    }

    template <int ROWS, int COLS>
    inline void printMat(const std::string &name, const modm::Matrix<float, ROWS, COLS> &m)
    {
        std::cout << name << "\n===============\n";
        print<ROWS, COLS>(m);
        std::cout << "\n===============\n";
    }
#endif

    void performUpdate(const modm::Matrix<float, INPUTS, 1> &y)
    {
        if (!initialized)
        {
            return;
        }

        // Predict state
        // TODO add control vector if necessary in the future
        xHat = A * xHat;
        P = A * P * At + Q;

        CPCtR = C * P * Ct + R;
        // Use CMSIS inverse since modm::Matrix doesn't support inverse
        arm_mat_inverse_f32(&CPCtRARM, &CPCtRInverseARM);
        K = P * Ct * CPCtRInverse;
        xHat = xHat + K * (y - C * xHat);
        P = (I - K * C) * P;
    }

    using StateVectorArray = float[STATES];
    const StateVectorArray &getStateMatrix() const { return xHat.element; }

private:
    /**
     * State transition matrix. For "transitioning" the previous `xHat` to `xHat`
     *
     * @note Also referred to as "F" in literature.
     */
    const modm::Matrix<float, STATES, STATES> A;

    /**
     * Transpose of A, computed at the beginning and stored
     * to speed up update step.
     */
    modm::Matrix<float, STATES, STATES> At;

    /**
     * Observation matrix. How we transform the input into the form
     * of the state matrix.
     *
     * @note Also referred to as "H" in literature.
     */
    const modm::Matrix<float, INPUTS, STATES> C;

    /**
     * Transpose of C.
     */
    modm::Matrix<float, STATES, INPUTS> Ct;

    /**
     * Covariance matrices
     */
    const modm::Matrix<float, STATES, STATES> Q;
    const modm::Matrix<float, INPUTS, INPUTS> R;

    /**
     * Predicted state matrix at the current time.
     *
     * Expectation of the actual state given \f$Y_{i - 1}\f$.
     */
    modm::Matrix<float, STATES, 1> xHat;

    /**
     * Estimate error covariance.
     *
     * The variance of the actual state given \f$Y_{i - 1}\f$.
     */
    modm::Matrix<float, STATES, STATES> P;

    /**
     * Initial error covariance
     */
    modm::Matrix<float, STATES, STATES> P0;

    /**
     */
    modm::Matrix<float, STATES, INPUTS> K;

    /**
     * Identity matrix created upon construction and stored to avoid
     * having to compute it each update step.
     */
    modm::Matrix<float, STATES, STATES> I;

    arm_matrix_instance_f32 CPCtRARM;
    arm_matrix_instance_f32 CPCtRInverseARM;

    /// C * P * C^t + R
    modm::Matrix<float, INPUTS, INPUTS> CPCtR;
    /// Inverse matrix computation of CPCtR
    modm::Matrix<float, INPUTS, INPUTS> CPCtRInverse;

    bool initialized = false;
};

}  // namespace tap::algorithms

#endif  // TAPROOT_KALMAN_FILTER_HPP_
