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

#include "kalman_filter.hpp"

#include <modm/math/matrix.hpp>

namespace aruwlib
{
namespace algorithms
{
Kalman::Kalman(uint8_t n, uint8_t m)
{
    // initialize all variables?
}

template<typename T, uint8_t ROWS, uint8_t COLUMNS>
modm::Matrix<T, ROWS, COLUMNS>
Kalman::inverse(modm::Matrix<T, ROWS, COLUMNS> matrix)
{
    // only used on square matrices
    return matrix;
}

modm::Matrix<float, n, 1> Kalman::filterData(modm::Matrix<float, n, 1> z)
{
    predictState();
    predictCovariance(p, f, q);
    calculateKalmanGain(k, p, h, r);
    updateState(x, k, z, hx);
    updateCovariance();
}

modm::Matrix<float, n, 1> Kalman::getLastFiltered() const { return x; }

void Kalman::reset()
{
   // I have no idea if this is correct but it didn't give me an error
   // also I don't think all the values need to be reset
   x = modm::Matrix<float, n, 1>().zeroMatrix();
   z = modm::Matrix<float, m, 1>().zeroMatrix();

   p = modm::Matrix<float, n, n>().zeroMatrix();
   q = modm::Matrix<float, n, n>().zeroMatrix();
   r = modm::Matrix<float, m, m>().zeroMatrix();

   k = modm::Matrix<float, n, m>().zeroMatrix();

   f = modm::Matrix<float, n, n>().zeroMatrix();
   h = modm::Matrix<float, m, n>().zeroMatrix();

   fx = modm::Matrix<float, n, 1>().zeroMatrix();
   hx = modm::Matrix<float, m, 1>().zeroMatrix();
}

modm::Matrix<float, n, 1> predictState()
{
    // x = f(x, u)
}

modm::Matrix<float, n, n> predictCovariance(
    modm::Matrix<float, n, n> p,
    modm::Matrix<float, n, n> f,
    modm::Matrix<float, n, n> q)
{
    // P = F * P * Ft + Q
    modm::Matrix<float, n, n> tmp = f;
    tmp *= p;
    f.transpose();
    tmp *= f;
    tmp += q;
    return tmp;
}

modm::Matrix<float, n, m> calculateKalmanGain(  // incomplete
    modm::Matrix<float, n, m> k,
    modm::Matrix<float, n, n> p,
    modm::Matrix<float, m, n> h,
    modm::Matrix<float, m, m> r
    )
{
    // K = P * H * (H * P * Ht + R)^-1
    k = p * (h);  // I don't understand the operator for multiplying matrices of diff sizes
    // mxn * nxn * n*m + m*m
    modm::Matrix<float, n, n> tmp = h; // h * p * ht + r
    // more multiplication
    Kalman::inverse(tmp);  // idk how to call this
    return k;
}

modm::Matrix<float, n, 1> updateState(  // incomplete
    modm::Matrix<float, n, 1> x,
    modm::Matrix<float, n, m> k,
    modm::Matrix<float, m, 1> z,
    modm::Matrix<float, m, 1> hx)
{
    // x = x + K * (z - h(x))
    modm::Matrix<float, n, 1> tmp = k * (z - hx);  // again, don't know how this works
    x += tmp;
    return x;
}

modm::Matrix<float, n, n> updateCovariance()
{
    // P = (I - K * H) * P * (I - K * H)t + K * R * Kt
}

}  // namespace algorithms

}  // namespace aruwlib