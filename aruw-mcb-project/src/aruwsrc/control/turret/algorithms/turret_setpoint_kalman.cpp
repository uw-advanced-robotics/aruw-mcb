/*
 * Copyright (c) 2020-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include "turret_setpoint_kalman.hpp"

namespace aruwsrc::control::turret::algorithms
{
TurretSetpointKalmanFilter::TurretSetpointKalmanFilter()
    : ekf(stateTransitionFunction,
          observationFunction,
          stateJacobianFunction,
          observationJacobianFunction,
          EKF_Q,
          EKF_R,
          EKF_P0)
{
}

void TurretSetpointKalmanFilter::initialize(tap::algorithms::WrappedFloat initialPosition)
{
    float x0[int(TrackerState::NUM_STATES)] = {initialPosition.getWrappedValue(), 0.0f};
    ekf.init(x0);
}
void TurretSetpointKalmanFilter::update(
    const tap::algorithms::WrappedFloat& measuredPosition,
    float dt)
{
    ekf.predict(dt);

    float predictedPos = ekf.getStateVectorAsMatrix()[int(TrackerState::POS)];

    float shortestPathError = measuredPosition.minDifference(predictedPos);

    EKF::InputVector z;
    z.data[int(TrackerInput::MEASURED_POS)] = predictedPos - shortestPathError;

    ekf.update(z);
}
float TurretSetpointKalmanFilter::getEstimatedPosition() const
{
    return ekf.getStateVectorAsMatrix()[int(TrackerState::POS)];
}

float TurretSetpointKalmanFilter::getEstimatedVelocity() const
{
    return ekf.getStateVectorAsMatrix()[int(TrackerState::VEL)];
}

void TurretSetpointKalmanFilter::stateTransitionFunction(
    const EKF::StateVector& x_prev,
    EKF::StateVector& x_pred,
    float dt)
{
    x_pred.data[int(TrackerState::POS)] =
        x_prev.data[int(TrackerState::POS)] + (x_prev.data[int(TrackerState::VEL)] * dt);
    x_pred.data[int(TrackerState::VEL)] = x_prev.data[int(TrackerState::VEL)];
}

void TurretSetpointKalmanFilter::observationFunction(
    const EKF::StateVector& x,
    EKF::InputVector& h_x)
{
    h_x.data[int(TrackerInput::MEASURED_POS)] = x.data[int(TrackerState::POS)];
}

void TurretSetpointKalmanFilter::stateJacobianFunction(
    const EKF::StateVector&,
    EKF::StateMatrix& F,
    float dt)
{
    F.data[0] = 1.0f;
    F.data[1] = dt;
    F.data[2] = 0.0f;
    F.data[3] = 1.0f;
}

void TurretSetpointKalmanFilter::observationJacobianFunction(
    const EKF::StateVector&,
    EKF::ObservationMatrix& H)
{
    H.data[0] = 1.0f;
    H.data[1] = 0.0f;
}

}  // namespace aruwsrc::control::turret::algorithms