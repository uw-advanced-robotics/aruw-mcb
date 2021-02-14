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

#include "odometry_1d.hpp"

#include <iostream>

#include <aruwlib/algorithms/extended_kalman_filter.hpp>

namespace aruwsrc
{
namespace algorithms
{
Odometry1D::Odometry1D(control::SentinelDriveSubsystem *sentinel, StateVector x)
    : sentinel(sentinel),
      z(MeasurementVector::zeroMatrix()),
      A(SquareStateMatrix::identityMatrix()),
      C(modm::Matrix<float, MEASUREMENTS, STATES>::zeroMatrix()),
      ekf(this,
          x,
          SquareStateMatrix::zeroMatrix(),
          SquareStateMatrix::zeroMatrix(),
          SquareMeasurementMatrix::zeroMatrix(),
          &Odometry1D::fFunction,
          &Odometry1D::jFFunction,
          &Odometry1D::hFunction,
          &Odometry1D::jHFunction)
{
    SquareStateMatrix Q = SquareStateMatrix::zeroMatrix();
    Q[0][0] = 0.05 * 0.05;
    Q[1][1] = 0.1 * 0.1;
    Q[2][2] = 0.01 * 0.01;
    ekf.setQ(Q);

    // [0.005, 0]
    // [0, 0.005]
    SquareMeasurementMatrix R = SquareMeasurementMatrix::zeroMatrix();
    R[0][0] = .005;
    R[1][1] = .005;
    ekf.setR(R);

    // [1, 0, 0]
    // [0, 1, 0]
    C[0][0] = 1;
    C[0][1] = 1;

    // [1, t, 1/2 * t^2]
    // [0, 1, t]
    // [0, 0, 1]
    A[0][1] = DELTA;
    A[0][2] = 0.5f * DELTA * DELTA;
    A[1][2] = DELTA;
}

const Odometry1D::StateVector &Odometry1D::fFunction(const Odometry1D::StateVector &x)
{
    Ax = A * x;
    return Ax;
}

const Odometry1D::SquareStateMatrix &Odometry1D::jFFunction(const Odometry1D::StateVector &)
{
    return A;
}

const Odometry1D::MeasurementVector &Odometry1D::hFunction(const Odometry1D::StateVector &x)
{
    Cx = C * x;
    return Cx;
}

const Odometry1D::MeasurementStateMatrix &Odometry1D::jHFunction(const Odometry1D::StateVector &)
{
    return C;
}

const Odometry1D::StateVector &Odometry1D::runIteration()
{
    z[0][0] = sentinel->absolutePosition();            // y
    z[1][0] = sentinel->getVelocityChassisRelative();  // vy
    return ekf.filterData(z);
}

const Odometry1D::StateVector &Odometry1D::getLastFiltered() { return ekf.getLastFiltered(); }

void Odometry1D::reset() { ekf.reset(); }
}  // namespace algorithms

}  // namespace aruwsrc
