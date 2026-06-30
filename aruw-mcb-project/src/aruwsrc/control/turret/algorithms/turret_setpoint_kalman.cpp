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
    // Initialize 3 states: [POS, VEL, ACCEL]
    float x0[int(TrackerState::NUM_STATES)] = {initialPosition.getWrappedValue(), 0.0f, 0.0f};
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

    int velVarInd = (static_cast<int>(TrackerInput::NUM_INPUTS) + 1) *
                    static_cast<int>(TrackerInput::MEASURED_VEL);
    int accVarInd = (static_cast<int>(TrackerInput::NUM_INPUTS) + 1) *
                    static_cast<int>(TrackerInput::MEASURED_ACC);
    auto& internal_R = ekf.getMeasurementCovariance();

    float velVar = internal_R[velVarInd];
    float accVar = internal_R[accVarInd];

    // Ignore unused variances
    internal_R[velVarInd] = 1e20f;
    internal_R[accVarInd] = 1e20f;

    ekf.update(z);

    internal_R[accVarInd] = accVar;
    internal_R[velVarInd] = velVar;
}

void TurretSetpointKalmanFilter::updateWithVelocity(
    const tap::algorithms::WrappedFloat& measuredPosition,
    float measuredVel,
    float dt)
{
    ekf.predict(dt);

    float predictedPos = ekf.getStateVectorAsMatrix()[int(TrackerState::POS)];

    float shortestPathError = measuredPosition.minDifference(predictedPos);

    EKF::InputVector z;
    z.data[int(TrackerInput::MEASURED_POS)] = predictedPos - shortestPathError;
    z.data[int(TrackerInput::MEASURED_VEL)] = measuredVel;

    int accVarInd = (static_cast<int>(TrackerInput::NUM_INPUTS) + 1) *
                    static_cast<int>(TrackerInput::MEASURED_ACC);
    auto& internal_R = ekf.getMeasurementCovariance();

    float accVar = internal_R[accVarInd];
    internal_R[accVarInd] = 1e20f;

    ekf.update(z);

    internal_R[accVarInd] = accVar;
}

void TurretSetpointKalmanFilter::updateWithAcceleration(
    const tap::algorithms::WrappedFloat& measuredPosition,
    float measuredVel,
    float measuredAcc,
    float dt)
{
    ekf.predict(dt);

    float predictedPos = ekf.getStateVectorAsMatrix()[int(TrackerState::POS)];

    float shortestPathError = measuredPosition.minDifference(predictedPos);

    EKF::InputVector z;
    z.data[int(TrackerInput::MEASURED_POS)] = predictedPos - shortestPathError;
    z.data[int(TrackerInput::MEASURED_VEL)] = measuredVel;
    z.data[int(TrackerInput::MEASURED_ACC)] = measuredAcc;

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

float TurretSetpointKalmanFilter::getEstimatedAcceleration() const
{
    return ekf.getStateVectorAsMatrix()[int(TrackerState::ACCEL)];
}

void TurretSetpointKalmanFilter::stateTransitionFunction(
    const EKF::StateVector& x_prev,
    EKF::StateVector& x_pred,
    float dt)
{
    // pos = pos + vel*dt + 0.5*accel*dt^2
    x_pred.data[int(TrackerState::POS)] = x_prev.data[int(TrackerState::POS)] +
                                          (x_prev.data[int(TrackerState::VEL)] * dt) +
                                          (0.5f * x_prev.data[int(TrackerState::ACCEL)] * dt * dt);

    // vel = vel + accel*dt
    x_pred.data[int(TrackerState::VEL)] =
        x_prev.data[int(TrackerState::VEL)] + (x_prev.data[int(TrackerState::ACCEL)] * dt);

    // accel = accel (constant acceleration assumption between steps)
    x_pred.data[int(TrackerState::ACCEL)] = x_prev.data[int(TrackerState::ACCEL)];
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
    // Row 1: d(pos_pred) / d(pos, vel, accel)
    F.data[0] = 1.0f;
    F.data[1] = dt;
    F.data[2] = 0.5f * dt * dt;

    // Row 2: d(vel_pred) / d(pos, vel, accel)
    F.data[3] = 0.0f;
    F.data[4] = 1.0f;
    F.data[5] = dt;

    // Row 3: d(accel_pred) / d(pos, vel, accel)
    F.data[6] = 0.0f;
    F.data[7] = 0.0f;
    F.data[8] = 1.0f;
}

void TurretSetpointKalmanFilter::observationJacobianFunction(
    const EKF::StateVector&,
    EKF::ObservationMatrix& H)
{
    // d(measured_pos) / d(pos, vel, accel)
    H.data[0] = 1.0f;
    H.data[1] = 0.0f;
    H.data[2] = 0.0f;
}

}  // namespace aruwsrc::control::turret::algorithms