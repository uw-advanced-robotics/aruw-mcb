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

#ifndef ODOMETRY_2D_HPP_
#define ODOMETRY_2D_HPP_

#include <aruwlib/Drivers.hpp>
#include <aruwlib/algorithms/extended_kalman_filter.hpp>
#include <modm/math/matrix.hpp>

#include "aruwsrc/control/chassis/chassis_subsystem.hpp"

namespace aruwsrc
{
namespace algorithms
{
class Odometry2D
{
public:
    static constexpr uint8_t STATES = 6;  // <x, y, vx, vy, θz, ωz> all world relative
    static constexpr uint8_t MEASUREMENTS =
        5;  // <vx_chassis, vy_chassis, θz_world, ωz_wheels, ωz_world>

    static constexpr float DELTA = 1;

    using StateVector = modm::Matrix<float, STATES, 1>;
    using SquareStateMatrix = modm::Matrix<float, STATES, STATES>;
    using SquareMeasurementMatrix = modm::Matrix<float, MEASUREMENTS, MEASUREMENTS>;
    using MeasurementVector = modm::Matrix<float, MEASUREMENTS, 1>;
    using MeasurementStateMatrix = modm::Matrix<float, MEASUREMENTS, STATES>;
    using ExtendedKalmanFilter =
        aruwlib::algorithms::ExtendedKalmanFilter<STATES, MEASUREMENTS, Odometry2D>;

    Odometry2D(
        aruwlib::Drivers *drivers,
        chassis::ChassisSubsystem *chassis,
        StateVector x = StateVector::zeroMatrix());

    const StateVector &runIteration();  // run EKF

    SquareStateMatrix configureForUpdate();  // update F with DELTA

    const StateVector &getLastFiltered();  // get last x

    void reset(ExtendedKalmanFilter ekf);  // reset EKF

    const StateVector &fFunction(const StateVector &x);

    const SquareStateMatrix &jFFunction(const StateVector &);

    const MeasurementVector &hFunction(const StateVector &x);

    const MeasurementStateMatrix &jHFunction(const StateVector &x);

private:
    aruwlib::Drivers *drivers;
    chassis::ChassisSubsystem *chassis;

    MeasurementVector z;  ///< measurement vector

    MeasurementVector hx;
    MeasurementStateMatrix jH;
    SquareStateMatrix F;  ///< state transition matrix
    StateVector Fx;       /// State transition matrix multiplied by x

    ExtendedKalmanFilter ekf;
};  // class Odometry2D

}  // namespace algorithms

}  // namespace aruwsrc

#endif  // ODOMETRY_2D_HPP_
