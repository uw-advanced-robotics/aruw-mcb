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
/**
 * 
 * 
 */
class Kalman
{
public:
    /**
     * Initializes a Kalman Filter with the given number of state and measured values.
     * 
     * @param[in] n number of state values.
     * @param[in] m number of measured values.
     */
    Kalman(uint8_t n, uint8_t m);   // idk if these should actually be the parameters

    /**
     * Performs one iteration of the Kalman Filter algorithm.
     * 
     * @param[in] z the data to be filtered.
     * @return the filtered data.
     */
    modm::Matrix<float, n, 1> filterData(modm::Matrix<float, n, 1> z);

    /**
     * Finds the inverse of a matrix.
     * 
     * @param[in] matrix the matrix to be inverted.
     * @return the inverted matrix.
     */
    template<typename T, uint8_t ROWS, uint8_t COLUMNS>
    modm::Matrix<T, ROWS, COLUMNS>
    inverse(modm::Matrix<T, ROWS, COLUMNS> matrix);

    /// Returns the last filtered data point.
    modm::Matrix<float, n, 1> getLastFiltered() const;

    /// Resets the covariances and predictions.
    void reset();

private:
    // idk if the variable names should be capitalized or not
    modm::Matrix<float, n, 1> x;  ///< state vector
    modm::Matrix<float, m, 1> z;  ///< measurement vector
   
    modm::Matrix<float, n, n> p;  ///< prediction error covariance
    modm::Matrix<float, n, n> q;  ///< process noise covariance
    modm::Matrix<float, m, m> r;  ///< measurement error covariance

    modm::Matrix<float, n, m> k;  ///< Kalman gain

    modm::Matrix<float, n, n> f;  ///< Jacobian of process model
    modm::Matrix<float, m, n> h;  ///< Jacobian of measurement model

    // I think this is why Simon "Dlevy" didn't use the first KF equation
    // because he expects the user to input the result from that
    modm::Matrix<float, n, 1> fx; ///< output of state-transition function (predicted state?)
    modm::Matrix<float, m, 1> hx; ///< output of measurement function
};                                // class Kalman

}  // namespace algorithms

}  // namespace aruwlib

#endif  // KALMAN_FILTER_HPP_