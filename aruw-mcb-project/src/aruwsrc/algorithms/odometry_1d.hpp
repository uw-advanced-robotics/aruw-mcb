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

#ifndef __ODOMETRY_1D_HPP__
#define __ODOMETRY_1D_HPP__

#include <aruwlib/algorithms/extended_kalman_filter.hpp>
#include <modm/math/matrix.hpp>

namespace aruwsrc
{
namespace algorithms
{
class Odometry1D
{
public:
    static constexpr uint8_t STATES = 3;  // x, vx, ax
    static constexpr uint8_t MEASUREMENTS = 2;  // x, vx

    static constexpr float DELTA = 1;

    using StateVector = modm::Matrix<float, STATES, 1>;
    using SquareStateMatrix = modm::Matrix<float, STATES, STATES>;
    using SquareMeasurementMatrix = modm::Matrix<float, MEASUREMENTS, MEASUREMENTS>;
    using MeasurementVector = modm::Matrix<float, MEASUREMENTS, 1>;

    Odometry1D(StateVector x) : x(x)
    {
        F = SquareStateMatrix::identityMatrix();  // not sure if this should start at t = 0

        H = modm::Matrix<float, MEASUREMENTS, STATES>::zeroMatrix();
        H[0][0] = 1;  // [1, 0, 0]
        H[0][1] = 1;  // [0, 1, 0]

        // I got these values from the CV code
        Q = SquareStateMatrix::zeroMatrix();
        Q[0][0] = 0.05 * 0.05;
        Q[1][1] = 0.1 * 0.1;
        Q[2][2] = 0.01 * 0.01;

        R = SquareMeasurementMatrix::identityMatrix();
        R[0][0] = 0.005;

        P = SquareStateMatrix::zeroMatrix();
    }

    void initialize();  // initialize KF

    StateVector runIteration();  // run KF

    SquareStateMatrix configureForUpdate();  // update F with DELTA

    bool shouldReset();  // check if KF needs to be reset

    const StateVector &getLastFiltered();  // get last x

    // reset method

    StateVector fFunction(const StateVector &x);
    SquareStateMatrix jFFunction(const StateVector &);
    MeasurementVector hFunction(const StateVector &x);
    modm::Matrix<float, MEASUREMENTS, STATES> jHFunction(const StateVector &);

private:
    StateVector x;  ///< state vector
    SquareStateMatrix F;
    modm::Matrix<float, MEASUREMENTS, STATES> H;
    SquareStateMatrix Q;        ///< process noise covariance
    SquareMeasurementMatrix R;  ///< measurement error covariance
    SquareStateMatrix P;
};  // class Odometry1D

}  // namespace algorithms

}  // namespace aruwsrc

#endif  // __ODOMETRY_1D_HPP__