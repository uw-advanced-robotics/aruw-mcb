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
    using StateVector = modm::Matrix<float, N, 1>;
    using SquareStateMatrix = modm::Matrix<float, N, N>;
    using MeasurementVector = modm::Matrix<float, M, 1>;
    using SquareMeasurementMatrix = modm::Matrix<float, M, M>;

    using FFunc = const StateVector &(T::*)(const StateVector &);
    using JFFunc = const SquareStateMatrix &(T::*)(const StateVector &);
    using HFunc = const MeasurementVector &(T::*)(const StateVector &);
    using JHFunc = const modm::Matrix<float, M, N> &(T::*)(const StateVector &x);

    /**
     * Initializes a Kalman Filter.
     *
     * @param[in] x the initial state estimate.
     * @param[in] p the initial prediction error covariance estimate.
     * @param[in] q the process noise covariance.
     * @param[in] r the measurement error covariance.
     * @param[in] fFunction the process function.
     * @param[in] jFFunction The jacobian of the process function.
     * @param[in] hFunction the measurement function.
     * @param[in] jHFunction The jacobian of the measurement function.
     */
    ExtendedKalmanFilter(
        T *odom,
        StateVector x,
        SquareStateMatrix p,
        SquareStateMatrix q,
        SquareMeasurementMatrix r,
        FFunc fFunction,
        JFFunc jFFunction,
        HFunc hFunction,
        JHFunc jHFunction)
        : x(x),
          p(p),
          q(q),
          r(r),
          k(modm::Matrix<float, N, M>::zeroMatrix()),
          i(SquareStateMatrix::identityMatrix()),
          odom(odom),
          fFunc(fFunction),
          jFFunc(jFFunction),
          hFunc(hFunction),
          jHFunc(jHFunction)
    {
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
        x = StateVector().zeroMatrix();
        p = SquareStateMatrix().zeroMatrix();
        k = modm::Matrix<float, N, M>().zeroMatrix();
    }

private:
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
        modm::Matrix<float, M, N> jH = (odom->*jHFunc)(x);
        modm::Matrix<float, N, M> jHTrans = jH.asTransposed();
        SquareMeasurementMatrix innovationCovariance = (jH * p) * jHTrans + r;
        SquareMeasurementMatrix innovationCovarianceInverse;
        inverse(innovationCovariance, &innovationCovarianceInverse);
        k = (p * jHTrans) * innovationCovarianceInverse;

        // P = (I - K * H) * P
        p = (i - k * jH) * p;
    }

    StateVector x;  /// state vector

    SquareStateMatrix p;  /// prediction error covariance

    SquareStateMatrix q;        /// process noise covariance
    SquareMeasurementMatrix r;  /// measurement error covariance

    modm::Matrix<float, N, M> k;  /// Kalman gain

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
