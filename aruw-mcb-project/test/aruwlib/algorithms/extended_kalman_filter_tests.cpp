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

/**
 * This is a test setup that should theoretically be a linear kalman filter
 * (it uses the EKF class in such a way that the transition and measurement
 * matrices are linear) that can be used for the sentinel's odom calculations.
 *
 * The sentinel this year operates in a single dimension, so the state vector
 * looks like this:
 * [ y   ]
 * [ v_y ]
 * [ a_y ]
 *
 * The transition matrix looks like this:
 *
 * [ 1    delta    1/2 * delta^2 ]
 * [ 0    1        delta         ]
 * [ 0    0        1             ]
 *
 * There are two measurements. The measurement vector looks like this:
 * [ y   ]
 * [ v_y ]
 *
 * The measurement matrix looks like this:
 * [ 1 0 0 ]
 * [ 0 1 0 ]
 */

#include <aruwlib/algorithms/extended_kalman_filter.hpp>
#include <gtest/gtest.h>

using namespace aruwlib::algorithms;

static constexpr uint8_t STATES = 3;
static constexpr uint8_t MEASUREMENTS = 2;

static constexpr float DELTA = 1;

using SquareStateMatrix = modm::Matrix<float, STATES, STATES>;
using StateVector = modm::Matrix<float, STATES, 1>;
using SquareMeasurementMatrix = modm::Matrix<float, MEASUREMENTS, MEASUREMENTS>;
using MeasurementVector = modm::Matrix<float, MEASUREMENTS, 1>;

static SquareStateMatrix Fk;
static modm::Matrix<float, MEASUREMENTS, STATES> Hk;

static void init()
{
    Fk = SquareStateMatrix::identityMatrix();
    Fk[0][1] = DELTA;
    Fk[0][2] = 0.5f * DELTA * DELTA;
    Fk[1][2] = DELTA;

    Hk = modm::Matrix<float, MEASUREMENTS, STATES>::zeroMatrix();
    Hk[0][0] = 1;
    Hk[1][1] = 1;
}

static void runIteration(
    ExtendedKalmanFilter<STATES, MEASUREMENTS> &ekf,
    const MeasurementVector &z)
{
    auto x = ekf.filterData(z);
    std::cout << x[0][0] << ", " << x[1][0] << ", " << x[2][0] << std::endl;
}

static StateVector fFunction(const StateVector &x) { return Fk * x; }

static SquareStateMatrix jFFunction(const StateVector &) { return Fk; }

static MeasurementVector hFunction(const StateVector &x) { return Hk * x; }

static modm::Matrix<float, MEASUREMENTS, STATES> jHFunction(const StateVector &) { return Hk; }

TEST(ExtendedKalmanFilter, coolTest)
{
    init();
    StateVector x = StateVector::zeroMatrix();
    SquareStateMatrix p = SquareStateMatrix::zeroMatrix();
    SquareStateMatrix q = SquareStateMatrix::zeroMatrix();
    SquareMeasurementMatrix r = SquareMeasurementMatrix::zeroMatrix();
    MeasurementVector z = MeasurementVector::zeroMatrix();

    ExtendedKalmanFilter<STATES, MEASUREMENTS>
        ekf(x, p, q, r, fFunction, jFFunction, hFunction, jHFunction);

    z[0][0] = 100;
    z[1][0] = 10;

    for (int i = 0; i < 100; i++)
    {
        runIteration(ekf, z);
    }
}
