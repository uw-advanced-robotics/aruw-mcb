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
#include <aruwlib/algorithms/math_user_utils.hpp>

using namespace aruwlib::algorithms;

namespace aruwsrc
{
namespace algorithms
{
Odometry2D::Odometry2D(aruwlib::Drivers *drivers, chassis::ChassisSubsystem *chassis, StateVector x)
    : drivers(drivers),
      chassis(chassis),
      z(MeasurementVector::zeroMatrix()),
      hx(MeasurementVector::zeroMatrix()),
      jH(MeasurementStateMatrix::zeroMatrix()),
      F(SquareStateMatrix::identityMatrix()),
      ekf(this,
          x,
          SquareStateMatrix::zeroMatrix(),
          SquareStateMatrix::zeroMatrix(),
          SquareMeasurementMatrix::zeroMatrix(),
          &Odometry2D::fFunction,
          &Odometry2D::jFFunction,
          &Odometry2D::hFunction,
          &Odometry2D::jHFunction)
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

    SquareStateMatrix Q = SquareStateMatrix::identityMatrix();
    ekf.setQ(Q);

    SquareMeasurementMatrix R = SquareMeasurementMatrix::identityMatrix();
    ekf.setR(R);

    jH[2][4] = 1;
    jH[3][5] = 1;
    jH[4][5] = 1;
}

const Odometry2D::StateVector &Odometry2D::fFunction(const Odometry2D::StateVector &x)
{
    return Fx = F * x;
    return Fx;
}

const Odometry2D::SquareStateMatrix &Odometry2D::jFFunction(const Odometry2D::StateVector &)
{
    return F;
}

const Odometry2D::MeasurementVector &Odometry2D::hFunction(const Odometry2D::StateVector &x)
{
    float cosYaw = radiansToDegrees(cos(x[4][0]));
    float sinYaw = radiansToDegrees(sin(x[4][0]));
    hx[0][0] = x[2][0] * cosYaw - x[3][0] * sinYaw;
    hx[1][0] = x[2][0] * sinYaw + x[3][0] * cosYaw;
    hx[2][0] = x[4][0];
    hx[3][0] = x[5][0];
    hx[4][0] = x[5][0];
    return hx;
}

const Odometry2D::MeasurementStateMatrix &Odometry2D::jHFunction(const Odometry2D::StateVector &x)
{
    float cosYaw = radiansToDegrees(cos(x[4][0]));
    float sinYaw = radiansToDegrees(sin(x[4][0]));
    jH[0][2] = cosYaw;
    jH[0][3] = -sinYaw;
    jH[0][4] = -x[2][0] * sinYaw - x[3][0] * cosYaw;
    jH[1][2] = sinYaw;
    jH[1][3] = cosYaw;
    jH[1][4] = x[2][0] * cosYaw - x[3][0] * sinYaw;
    return jH;
}

const Odometry2D::StateVector &Odometry2D::runIteration()
{
    modm::Matrix<float, 3, 1> velocities = chassis->getActualVelocityChassisRelative();
    z[0][0] = velocities[0][0];
    z[1][0] = velocities[1][0];
    z[2][0] = drivers->mpu6500.getYaw();
    z[3][0] = velocities[2][0];
    z[4][0] = drivers->mpu6500.getGz();
    return ekf.filterData(z);
}

const Odometry2D::StateVector &Odometry2D::getLastFiltered() { return ekf.getLastFiltered(); }

void Odometry2D::reset(ExtendedKalmanFilter ekf) { ekf.reset(); }

}  // namespace algorithms

}  // namespace aruwsrc
