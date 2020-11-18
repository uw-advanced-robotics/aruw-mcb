/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include <modm/math/matrix.hpp>

namespace aruwlib
{
namespace algorithms
{
template <uint8_t N, uint8_t M>
class KalmanFilter
{
public:
    /**
     * Initializes a Kalman Filter.
     * 
     * @param[in] x the initial state estimate.
     * @param[in] p the initial prediction error covariance estimate.
     * @param[in] q the process noise covariance.
     * @param[in] r the measurement error covariance.
     */
    KalmanFilter(modm::Matrix<float, N, 1> x, modm::Matrix<float, N, N> p,
        modm::Matrix<float, N, N> q, modm::Matrix<float, M, M> r)
        : x(x),
          z(modm::Matrix<float, N, 1>::zeroMatrix()),
          p(p),
          q(q),
          r(r),
          k(modm::Matrix<float, N, M>::zeroMatrix()),
          f(modm::Matrix<float, N, N>::zeroMatrix()),
          h(modm::Matrix<float, M, N>::zeroMatrix()),
          fx(modm::Matrix<float, N, 1>::zeroMatrix()),
          hx(modm::Matrix<float, M, 1>::zeroMatrix())
    {
    }
    
    /**
     * Performs one iteration of the Kalman Filter algorithm.
     * 
     * @param[in] z the data to be filtered.
     * @return the filtered data.
     */
    modm::Matrix<float, N, 1> filterData(modm::Matrix<float, N, 1> z)
    {
        fx = predictState();
        // update h(x)?
        p = predictCovariance(p, f, q);
        k = calculateKalmanGain(k, p, h, r);
        x = updateState(fx, k, z, hx);
        p = updateCovariance();
    }

    /**
     * Finds the inverse of a matrix.
     * 
     * @param[in] matrix the matrix to be inverted.
     * @return the inverted matrix.
     */
    template<typename T, uint8_t ROWS, uint8_t COLUMNS>
    modm::Matrix<T, ROWS, COLUMNS> inverse(modm::Matrix<T, ROWS, COLUMNS> matrix)
    {
        // only used on square matrices
        return matrix;
    }

    /// Returns the last filtered data point.
    modm::Matrix<float, N, 1> getLastFiltered() const { return x; }

    /// Resets the covariances and predictions.
    void reset()
    {
        x = modm::Matrix<float, N, 1>().zeroMatrix();
        z = modm::Matrix<float, M, 1>().zeroMatrix();

        p = modm::Matrix<float, N, N>().zeroMatrix();

        k = modm::Matrix<float, N, M>().zeroMatrix();

        fx = modm::Matrix<float, N, 1>().zeroMatrix();
        hx = modm::Matrix<float, M, 1>().zeroMatrix();
    }

    modm::Matrix<float, N, 1> predictState()
    {
        // x = f(x, u)
    }

    modm::Matrix<float, N, N> predictCovariance(
        modm::Matrix<float, N, N> p,
        modm::Matrix<float, N, N> f,
        modm::Matrix<float, N, N> q)
    {
        // P = F * P * Ft + Q
        return f * p * f.asTransposed() + q;
    }

    modm::Matrix<float, N, M> calculateKalmanGain(
        modm::Matrix<float, N, M> k,
        modm::Matrix<float, N, N> p,
        modm::Matrix<float, M, N> h,
        modm::Matrix<float, M, M> r
        )
    {
        // K = P * Ht * (H * P * Ht + R)^-1
        modm::Matrix<float, n, n> innovationCovariance = (h * p * h.asTransposed() + r);
        innovationCovariance = inverse(innovationCovariance);
        return p * h.asTransposed() * innovationCovariance;
    }

    modm::Matrix<float, N, 1> updateState(
        modm::Matrix<float, N, 1> fx,
        modm::Matrix<float, N, M> k,
        modm::Matrix<float, M, 1> z,
        modm::Matrix<float, M, 1> hx)
    {
        // x = x + K * (z - h(x))
        return fx + k * (z - hx);  // I think fx should be used instead of x because fx is the previous predicted state but I could be wrong
    }

    modm::Matrix<float, n, n> updateCovariance()
    {
        // P = (I - K * H) * P * (I - K * H)t + K * R * Kt
    }

private:
    modm::Matrix<float, N, 1> x;  ///< state vector
    modm::Matrix<float, N, 1> z;  ///< measurement vector
   
    modm::Matrix<float, N, N> p;  ///< prediction error covariance
    modm::Matrix<float, N, N> q;  ///< process noise covariance
    modm::Matrix<float, M, M> r;  ///< measurement error covariance

    modm::Matrix<float, N, M> k;  ///< Kalman gain

    modm::Matrix<float, N, N> f;  ///< Jacobian of process model
    modm::Matrix<float, M, N> h;  ///< Jacobian of measurement model

    modm::Matrix<float, N, 1> fx; ///< output of state-transition function (predicted state)
    modm::Matrix<float, M, 1> hx; ///< output of measurement function (used in state update equation)
};                                // class Kalman

}  // namespace algorithms

}  // namespace aruwlib

#endif  // KALMAN_FILTER_HPP_