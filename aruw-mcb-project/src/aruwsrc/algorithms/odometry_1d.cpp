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

#include "odometry_1d.hpp"

#include <aruwlib/algorithms/extended_kalman_filter.hpp>

namespace aruwsrc
{
namespace algorithms
{
Odometry1D::ExtendedKalmanFilter Odometry1D::initialize()
{
    return Odometry1D::ExtendedKalmanFilter (this, x, P, Q, R, fFunction, jFFunction, hFunction, jHFunction);
}

Odometry1D::StateVector Odometry1D::fFunction(const Odometry1D::StateVector &x) { return F * x; }

Odometry1D::SquareStateMatrix Odometry1D::jFFunction(const Odometry1D::StateVector &) { return F; }

Odometry1D::MeasurementVector Odometry1D::hFunction(const Odometry1D::StateVector &x) { return H * x; }

modm::Matrix<float, Odometry1D::MEASUREMENTS, Odometry1D::STATES>
    Odometry1D::jHFunction(const Odometry1D::StateVector &) { return H; }

Odometry1D::StateVector Odometry1D::runIteration(ExtendedKalmanFilter ekf)
{
    z[0][0] = sentinel->absolutePosition() * 1000;     // y, converted from mm to m
    z[1][0] = sentinel->getVelocityChassisRelative();  // vy
    x = ekf.filterData(z);
    return x;
}

Odometry1D::SquareStateMatrix Odometry1D::configureForUpdate()
{
    // [1, t, 1/2t^2]
    // [0, 1, t]
    // [0, 0, 1]
    F[0][1] += DELTA;
    F[0][2] += 0.5 * DELTA * DELTA;
    F[1][2] += DELTA;
    return F;
}

const Odometry1D::StateVector &Odometry1D::getLastFiltered() { return x; }

void Odometry1D::reset(ExtendedKalmanFilter ekf) { ekf.reset(); }

}  // namespace algorithms

}  // namespace aruwsrc
