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
#include "aruwlib/algorithms/math_user_utils.hpp"

#include <aruwlib/algorithms/extended_kalman_filter.hpp>

using namespace aruwlib::algorithms;

namespace aruwsrc
{
namespace algorithms
{
Odometry2D::ExtendedKalmanFilter Odometry2D::initialize()
{
    return ExtendedKalmanFilter (this, x, P, Q, R, fFunction, jFFunction, hFunction, jHFunction);
}

Odometry2D::StateVector Odometry2D::fFunction(const Odometry2D::StateVector &x) { return F * x; }

Odometry2D::SquareStateMatrix Odometry2D::jFFunction(const Odometry2D::StateVector &) { return F; }

Odometry2D::MeasurementVector Odometry2D::hFunction(const Odometry2D::StateVector &x)
{
    float cosYaw = radiansToDegrees(cos(x[4][0]));
    float sinYaw = radiansToDegrees(sin(x[4][0]));
    MeasurementVector hx = MeasurementVector::zeroMatrix();
    hx[0][0] = x[2][0] * cosYaw - x[3][0] * sinYaw;
    hx[1][0] = x[2][0] * sinYaw + x[3][0] * cosYaw;
    hx[2][0] = x[4][0];
    hx[3][0] = x[5][0];
    hx[4][0] = x[5][0];
    return hx;
}

Odometry2D::MeasurementStateMatrix Odometry2D::jHFunction(const Odometry2D::StateVector &x)
{
    float cosYaw = radiansToDegrees(cos(x[4][0]));
    float sinYaw = radiansToDegrees(sin(x[4][0]));
    MeasurementStateMatrix jH = MeasurementStateMatrix::zeroMatrix();
    jH[0][2] = cosYaw;
    jH[0][3] = -sinYaw;
    jH[0][4] = -x[2][0] * sinYaw - x[3][0] * cosYaw;
    jH[1][2] = sinYaw;
    jH[1][3] = cosYaw;
    jH[1][4] = x[2][0] * cosYaw - x[3][0] * sinYaw;
    jH[2][4] = 1;
    jH[3][5] = 1;
    jH[4][5] = 1;
    return jH;
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
