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

#include "odometry_2d.hpp"

#include <aruwlib/algorithms/extended_kalman_filter.hpp>

namespace aruwsrc
{
namespace algorithms
{
Odometry2D::ExtendedKalmanFilter Odometry2D::initialize()
{
    return Odometry2D::ExtendedKalmanFilter (this, x, P, Q, R, fFunction, jFFunction, hFunction, jHFunction);
}

Odometry2D::StateVector Odometry2D::fFunction(const Odometry2D::StateVector &x) { return F * x; }

Odometry2D::SquareStateMatrix Odometry2D::jFFunction(const Odometry2D::StateVector &) { return F; }

Odometry2D::MeasurementVector Odometry2D::hFunction(const Odometry2D::StateVector &x)
{
    // to do: update with measurement function
    Odometry2D::MeasurementVector hx = Odometry2D::MeasurementVector::zeroMatrix();
    // vx_world:  x[2][0]
    // vy_world:  x[3][0]
    // yaw_world: x[4][0]
    // wz_world:  x[5][0]
    return hx;
}

modm::Matrix<float, Odometry2D::MEASUREMENTS, Odometry2D::STATES>
    Odometry2D::jHFunction(const Odometry2D::StateVector &)
{
    // to do: update with H Jacobian
    return H;
}

Odometry2D::StateVector Odometry2D::runIteration(ExtendedKalmanFilter ekf)
{
    // to do: put data in measurement vector
    x = ekf.filterData(z);
    return x;
}

Odometry2D::SquareStateMatrix Odometry2D::configureForUpdate()
{
    // [1, 0, t, 0, 0, 0]
    // [0, 1, 0, t, 0, 0]
    // [0, 0, 1, 0, 0, 0]
    // [0, 0, 0, 1, 0, 0]
    // [0, 0, 0, 0, 1, t]
    // [0, 0, 0, 0, 0, 1]
    F[0][2] += DELTA;
    F[1][3] += DELTA;
    F[4][5] += DELTA;
    return F;
}

const Odometry2D::StateVector &Odometry2D::getLastFiltered() { return x; }

void Odometry2D::reset(ExtendedKalmanFilter ekf) { ekf.reset(); }

}  // namespace algorithms

}  // namespace aruwsrc
