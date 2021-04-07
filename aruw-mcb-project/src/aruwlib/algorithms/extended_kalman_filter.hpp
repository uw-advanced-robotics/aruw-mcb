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

#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_

#include <aruw-mcb-project/modm/ext/cmsis/dsp/arm_math.h>
#include <modm/architecture/interface/assert.h>
#include <modm/math/matrix.hpp>

#include "math_user_utils.hpp"

namespace aruwlib
{
namespace algorithms
{
/**
 * This extended kalman filter implementation is defined by the following set of equations.
 *
 * The model and observation step. This is the model that we are using to filter incoming
 * data.
 * \f{eqnarray*}{
 * x_k & = & f(x_{k - 1}) + w_{k - 1}\\
 * z_k & = & h(x-k) + v_k
 * \f}
 *
 * Each iteration, first, the forcasted prediction for x is computed (represented by $x$
 * in code):
 * \f{eqnarray*}{
 * x_k & \approx & f(x_{k - 1}^a)
 * \f}
 *
 * Next the forcasted prediction matrix is computed (represented by $p$ in code):
 * \f{eqnarray*}{
 * P_k^f & = & J_f(x_{k - 1}^a) P_{k - 1} J_{f}^T (x_{k - 1}^a) + Q_{k - 1}
 * \f}
 * Note that \f$J_f(x_{k - 1}^a)\f$ is $jFFunc(x)$ in code.
 *
 * Next is the data assimilation step/corrector, where an actual x is predicted (represented
 * by $x$ in code), the kalman gain is predicted for the next iteration, and a new P
 * matrix is computed.
 * \f{eqnarray*}{
 * x_k^a & \approx & x_k^f + K_k(z_k - h(x_k^f))\\
 * K_k & = & P_k^f J_h^T (x_k^f) \(J_h(x)_k^f) P_k^f J_h^T (x_k^f) + R_k\)^{-1}\\
 * P_k & = & \(I - K_k J_h (x_k^f)\) P_k^f
 * \f}
 * In the equation above,
 * - \f$x_k^a\f$ is $x$
 * - \f$h(x_k^f)\f$ is $hFunc(x)$
 * - \f$P_k^f\f$ is $p$
 * - \f$J_h^T (x_k^f)\f$ is $fHFunc(x)$
 *
 * @tparam N The number of states
 * @tparam M The number of measured states
 * @see https://www.cse.sc.edu/~terejanu/files/tutorialEKF.pdf
 */
template <uint8_t N, uint8_t M, typename T>
class ExtendedKalmanFilter
{
public:
    using StateVector = arm_matrix_instance_f32;
    using SquareStateMatrix = arm_matrix_instance_f32;
    using MeasurementVector = arm_matrix_instance_f32;
    using SquareMeasurementMatrix = arm_matrix_instance_f32;

    using FFunc = const StateVector &(T::*)(const StateVector &);
    using JFFunc = const SquareStateMatrix &(T::*)(const StateVector &);
    using HFunc = const MeasurementVector &(T::*)(const StateVector &);
    using JHFunc = const arm_matrix_instance_f32 &(T::*)(const StateVector &x);

    // Matrix addition overload.
    inline arm_matrix_instance_f32 operator+(
        const arm_matrix_instance_f32 lhs,
        const arm_matrix_instance_f32 rhs)
    {
        arm_matrix_instance_f32 sum;
        arm_mat_add_f32(*lhs, *rhs, sum);
        return sum;
    }

    // Matrix subtraction overload.
    inline arm_matrix_instance_f32 operator-(
        const arm_matrix_instance_f32 lhs,
        const arm_matrix_instance_f32 rhs)
    {
        arm_matrix_instance_f32 difference;
        arm_mat_sub_f32(*lhs, *rhs, difference);
        return difference;
    }

    // Matrix multiplication overload.
    inline arm_matrix_instance_f32 operator*(
        const arm_matrix_instance_f32 lhs,
        const arm_matrix_instance_f32 rhs)
    {
        arm_matrix_instance_f32 product;
        arm_mat_mult_f32(*lhs, *rhs, product);
        return product;
    }

    /**
     * Initializes a floating-point identity matrix.
     *
     * @param[in, out] S    points to the instance of the floating-point matrix structure.
     * @param[in] n         number of rows/columns in the matrix.
     */
    inline void initIdentityMatrix(arm_matrix_instance_f32 S, const u_int16_t n, float32_t *data)
    {
        data = {};
        for (int i = 0; i < n * n; i += n + 1)
        {
            data[i] = 1;
        }
        arm_mat_init_f32(S, n, n, data);
    }

    /**
     * Initializes a floating-point zero matrix.
     *
     * @param[in, out] S    points to the instance of the floating-point matrix structure.
     * @param[in] n         number of rows in the matrix.
     * @param[in] m         number of columns in the matrix.
     */
    inline void initZeroMatrix(
        arm_matrix_instance_f32 S,
        const u_int16_t n,
        const u_int16_t m,
        float32_t *data)
    {
        data = {};
        arm_mat_init_f32(S, n, m, data);
    }

    /**
     * Initializes a Kalman Filter.
     *
     * @param[in] xData the initial state estimate.
     * @param[in] pData the initial prediction error covariance estimate.
     * @param[in] qData the process noise covariance.
     * @param[in] rData the measurement error covariance.
     * @param[in] fFunction the process function.
     * @param[in] jFFunction The jacobian of the process function.
     * @param[in] hFunction the measurement function.
     * @param[in] jHFunction The jacobian of the measurement function.
     */
    ExtendedKalmanFilter(
        T *odom,
        float32_t xData[],
        float32_t pData[],
        float32_t qData[],
        float32_t rData[],
        FFunc fFunction,
        JFFunc jFFunction,
        HFunc hFunction,
        JHFunc jHFunction)
        : x(),
          p(),
          q(),
          r(),
          k(),
          i(),
          odom(odom),
          fFunc(fFunction),
          jFFunc(jFFunction),
          hFunc(hFunction),
          jHFunc(jHFunction)
    {
        std::copy(std::begin(xData), std::end(xData), std::begin(this->xData));
        std::copy(std::begin(pData), std::end(pData), std::begin(this->pData));
        std::copy(std::begin(qData), std::end(qData), std::begin(this->qData));
        std::copy(std::begin(rData), std::end(rData), std::begin(this->rData));
        arm_mat_init_f32(x, N, 1, this->xData);
        arm_mat_init_f32(p, N, N, this->pData);
        arm_mat_init_f32(q, N, N, this->qData);
        arm_mat_init_f32(r, M, M, this->rData);
        initZeroMatrix(k, N, M, this->kData);
        initIdentityMatrix(i, N, this->iData);
        modm_assert(
            fFunc != nullptr && jFFunc != nullptr && hFunc != nullptr && jHFunc != nullptr,
            "ekf",
            "nullptr functions");
    }

    void setQ(const SquareStateMatrix &newQ) { q = newQ; }
    void setR(const SquareMeasurementMatrix &newR) { r = newR; }

    /**
     * Performs one iteration of the Kalman Filter algorithm.
     *
     * @param[in] z the data to be filtered.
     * @return the filtered data.
     */
    const StateVector &filterData(const MeasurementVector &z)
    {
        predictState();
        predictCovariance();
        updateState(z);
        calculateKalmanGain();
        return x;
    }

    /// Returns the last filtered data point.
    inline const StateVector &getLastFiltered() const { return x; }

    /// Resets the covariances and predictions.
    void reset()
    {
        initZeroMatrix(x, N, 1);
        initZeroMatrix(p, N, N);
        initZeroMatrix(k, N, M);
    }

private:
    float32_t xData[N];      // state estimate
    float32_t pData[N * N];  // prediction error covariance data
    float32_t qData[N * N];  // process noise covariance data
    float32_t rData[M * M];  // measurement error covariance data
    float32_t kData[N * M];  // kalman gain
    float32_t iData[N * N];  // identity matrix

    /// Predicts the state at the next time step.
    inline void predictState()
    {
        // x = f(x)
        x = (odom->*fFunc)(x);
    }

    /// Predicts the prediction error covariance at the next time step.
    inline void predictCovariance()
    {
        // P = F * P * Ft + Q
        auto jF = (odom->*jFFunc)(x);
        p = jF * p * jF.asTransposed() + q;
    }

    /// Updates the state using the measurement.
    inline void updateState(const MeasurementVector &z)
    {
        // x = x + K * (z - h(x))
        x = x + k * (z - (odom->*hFunc)(x));
    }

    /// Calculates the Kalman Gain.
    inline void calculateKalmanGain()
    {
        // K = P * Ht * (H * P * Ht + R)^-1
        arm_matrix_instance_f32 jH = (odom->*jHFunc)(x);
        arm_matrix_instance_f32 jHTrans = jH.asTransposed();
        SquareMeasurementMatrix innovationCovariance = (jH * p) * jHTrans + r;
        SquareMeasurementMatrix innovationCovarianceInverse;
        arm_mat_inverse_f32(&innovationCovariance, &innovationCovarianceInverse);
        k = (p * jHTrans) * innovationCovarianceInverse;

        // P = (I - K * H) * P
        p = (i - k * jH) * p;
    }

    StateVector x;  /// state vector

    SquareStateMatrix p;  /// prediction error covariance

    SquareStateMatrix q;        /// process noise covariance
    SquareMeasurementMatrix r;  /// measurement error covariance

    arm_matrix_instance_f32 k;  /// Kalman gain

    SquareStateMatrix i;  /// An I matrix

    /// Pointer to object that contains function pointers for user defined process and measurement
    /// matrices
    T *odom;
    /// user-defined process function
    FFunc fFunc;
    /// user-defined jacobian of the process function
    JFFunc jFFunc;
    /// user-defined measurement function
    HFunc hFunc;
    /// user-defined jacobian of the measurement function
    JHFunc jHFunc;
};  // class ExtendedKalmanFilter

}  // namespace algorithms

}  // namespace aruwlib

#endif  // KALMAN_FILTER_HPP_
