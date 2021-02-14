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

#ifndef ODOMETRY_1D_HPP_
#define ODOMETRY_1D_HPP_

#include <aruwlib/algorithms/extended_kalman_filter.hpp>
#include <modm/math/matrix.hpp>

#include "aruwsrc/control/sentinel/sentinel_drive_subsystem.hpp"

namespace aruwsrc
{
namespace algorithms
{
class Odometry1D
{
public:
    static constexpr uint8_t STATES = 3;        // <y, vy, ay>
    static constexpr uint8_t MEASUREMENTS = 2;  // <y, vy>

    static constexpr float DELTA = 0.002f;

    using StateVector = modm::Matrix<float, STATES, 1>;
    using SquareStateMatrix = modm::Matrix<float, STATES, STATES>;
    using SquareMeasurementMatrix = modm::Matrix<float, MEASUREMENTS, MEASUREMENTS>;
    using MeasurementVector = modm::Matrix<float, MEASUREMENTS, 1>;
    using MeasurementStateMatrix = modm::Matrix<float, MEASUREMENTS, STATES>;
    using ExtendedKalmanFilter =
        aruwlib::algorithms::ExtendedKalmanFilter<STATES, MEASUREMENTS, Odometry1D>;

    Odometry1D(
        control::SentinelDriveSubsystem *sentinel,
        StateVector x = StateVector::zeroMatrix());

    const StateVector &runIteration();  // run EKF

    const StateVector &getLastFiltered();  // get last x

    void reset();  // reset EKF

    const StateVector &fFunction(const StateVector &x);

    const SquareStateMatrix &jFFunction(const StateVector &);

    const MeasurementVector &hFunction(const StateVector &x);

    const MeasurementStateMatrix &jHFunction(const StateVector &);

private:
    control::SentinelDriveSubsystem *sentinel;

    MeasurementVector z;  ///< measurement vector

    SquareStateMatrix A;       /// A matrix (as seen in a normal kalman filter implementation)
    StateVector Ax;            /// A matrix multiplied by x
    MeasurementStateMatrix C;  /// C matrix (as seen in a normal kalman filter implementation)
    MeasurementVector Cx;      /// C matrix multiplied by x

    ExtendedKalmanFilter ekf;
};  // class Odometry1D

}  // namespace algorithms

}  // namespace aruwsrc

#endif  // ODOMETRY_1D_HPP_
