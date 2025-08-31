/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "wheel_ekf_odometry.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"

#include <cmath>

namespace aruwsrc::algorithms::odometry
{

FourWheelEKFOdometry::FourWheelEKFOdometry(
    const tap::motor::DjiMotor *chassisMotors[4],
    tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
    tap::communication::sensors::imu::ImuInterface& imu,
    const modm::Vector2f initPos)
    : ekf(stateTransitionFunction,
          observationFunction,
          stateJacobianFunction,
          observationJacobianFunction,
          EKF_Q,
          EKF_R,
          EKF_P0),
      chassisYawObserver(chassisYawObserver),
      imu(imu),
      initPos(initPos),
      chassisAccelerationToMeasurementCovarianceInterpolator(
          CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT,
          MODM_ARRAY_SIZE(CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT))
{
    // Copy motor pointers to member array
    for (int i = 0; i < 4; i++) {
        this->chassisMotors[i] = chassisMotors[i];
    }
    reset();
}

void FourWheelEKFOdometry::reset()
{
    float initialX[int(OdomState::NUM_STATES)] = {
        initPos.x,  // POS_X
        0.0f,       // VEL_X
        0.0f,       // ACC_X
        initPos.y,  // POS_Y
        0.0f,       // VEL_Y
        0.0f,       // ACC_Y
        0.0f,       // BIAS_ACC_X
        0.0f        // BIAS_ACC_Y
    };
    ekf.init(initialX);
}

void FourWheelEKFOdometry::update()
{
    if (!chassisYawObserver.getChassisWorldYaw(&chassisYaw))
    {
        chassisYaw = 0;
        return;
    }

    uint32_t currentTime = tap::arch::clock::getTimeMilliseconds();
    float dt = prevTime == 0 ? DT : (currentTime - prevTime) / 1000.0f;
    dt = std::max(dt, 0.001f);  // Minimum dt to avoid division by zero
    prevTime = currentTime;

    // Get individual wheel velocities and convert to linear velocities
    for (int i = 0; i < 4; i++)
    {
        motorVel = chassisMotors[i]->getEncoder()->getVelocity(); // rad/s
        wheelLinearVel = (motorVel / WHEEL_CONFIGS[i].gearRatio) * WHEEL_CONFIGS[i].wheelRadius; // m/s
        
        // Calculate wheel velocity components in chassis frame based on wheel orientation
        float wheelAngle = WHEEL_CONFIGS[i].wheelOrientationToForwardRadians;
        
        z[int(OdomInput::VEL_X_1) + i * 2] = wheelLinearVel * cos(wheelAngle);
        z[int(OdomInput::VEL_Y_1) + i * 2] = wheelLinearVel * sin(wheelAngle);
    }

    // Get IMU acceleration data in chassis frame
    z[int(OdomInput::ACC_X)] = imu.getAx();
    z[int(OdomInput::ACC_Y)] = imu.getAy();

    // Rotate acceleration from MCB frame to the world frame
    tap::algorithms::rotateVector(
        &z[int(OdomInput::ACC_X)],
        &z[int(OdomInput::ACC_Y)],
        aruwsrc::serial::VisionCoprocessor::MCB_ROTATION_OFFSET + chassisYaw);

    // Calculate average chassis velocity for measurement covariance update
    modm::Vector2f chassisVelocity;
    chassisVelocity.x = (z[int(OdomInput::VEL_X_1)] + z[int(OdomInput::VEL_X_2)] + 
                        z[int(OdomInput::VEL_X_3)] + z[int(OdomInput::VEL_X_4)]) / 4.0f;
    chassisVelocity.y = (z[int(OdomInput::VEL_Y_1)] + z[int(OdomInput::VEL_Y_2)] + 
                        z[int(OdomInput::VEL_Y_3)] + z[int(OdomInput::VEL_Y_4)]) / 4.0f;

    // Update measurement covariance based on acceleration
    updateMeasurementCovariance(chassisVelocity);

    // Create measurement vector
    ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::InputVector measurement;
    for (int i = 0; i < int(OdomInput::NUM_INPUTS); i++)
    {
        measurement.data[i] = z[i];
    }

    // Perform the EKF update - prediction and correction
    ekf.performUpdate(measurement, dt);

    // Update the location and velocity accessor objects with values from the state vector
    updateChassisStateFromEKF(chassisYaw);
}

void FourWheelEKFOdometry::updateChassisStateFromEKF(float chassisYaw)
{
    const auto& x = ekf.getStateVectorAsMatrix();

    // update odometry velocity and orientation
    velocity.x = x[int(OdomState::VEL_X)];
    velocity.y = x[int(OdomState::VEL_Y)];

    // Set location
    location.setPosition(x[int(OdomState::POS_X)], x[int(OdomState::POS_Y)]);
    location.setOrientation(chassisYaw);
}

void FourWheelEKFOdometry::updateMeasurementCovariance(const modm::Vector2f& chassisVelocity)
{
    float chassisAccelerationMagnitude = std::sqrt(
        chassisVelocity.x * chassisVelocity.x + 
        chassisVelocity.y * chassisVelocity.y);

    auto& R = ekf.getMeasurementCovariance();

    float measurementCovariance = chassisAccelerationToMeasurementCovarianceInterpolator.interpolate(
        chassisAccelerationMagnitude);

    // Update wheel velocity measurement covariances
    for (int i = 0; i < 8; i++) // 4 wheels * 2 components each
    {
        R[i * int(OdomInput::NUM_INPUTS) + i] = measurementCovariance;
    }
}

void FourWheelEKFOdometry::overrideOdometryPosition(const float positionX, const float positionY)
{
    float initialX[int(OdomState::NUM_STATES)];
    const auto& currentState = ekf.getStateVectorAsMatrix();
    
    // Copy current state and update position
    for (int i = 0; i < int(OdomState::NUM_STATES); i++)
    {
        initialX[i] = currentState[i];
    }
    
    initialX[int(OdomState::POS_X)] = positionX;
    initialX[int(OdomState::POS_Y)] = positionY;
    
    ekf.init(initialX);
}

// Static function implementations for EKF

void FourWheelEKFOdometry::stateTransitionFunction(
    const ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::StateVector& x_prev,
    ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::StateVector& x_pred,
    float dt)
{
    // Enhanced nonlinear state transition model
    // Incorporates velocity-dependent process noise and acceleration constraints
    
    float vel_x = x_prev.data[int(OdomState::VEL_X)];
    float vel_y = x_prev.data[int(OdomState::VEL_Y)];
    float acc_x = x_prev.data[int(OdomState::ACC_X)];
    float acc_y = x_prev.data[int(OdomState::ACC_Y)];
    
    // Velocity magnitude for nonlinear effects
    float vel_magnitude = std::sqrt(vel_x * vel_x + vel_y * vel_y);
    
    // Position: Enhanced kinematic model with velocity-dependent corrections
    // Include second-order effects for high-speed motion
    float vel_correction_factor = 1.0f + VELOCITY_CORRECTION_COEFFICIENT * vel_magnitude;
    x_pred.data[int(OdomState::POS_X)] = x_prev.data[int(OdomState::POS_X)] + 
                                         vel_x * dt * vel_correction_factor + 
                                         0.5f * acc_x * dt * dt;
    
    x_pred.data[int(OdomState::POS_Y)] = x_prev.data[int(OdomState::POS_Y)] + 
                                         vel_y * dt * vel_correction_factor + 
                                         0.5f * acc_y * dt * dt;
    
    // Velocity: Enhanced model with acceleration constraints
    // Limit unrealistic acceleration changes
    float acc_change_x = acc_x * dt;
    float acc_change_y = acc_y * dt;
    
    // Apply acceleration limits (nonlinear constraint)
    float max_acc_change_dt = MAX_ACCELERATION_CHANGE_RATE * dt;
    if (std::abs(acc_change_x) > max_acc_change_dt) {
        acc_change_x = std::copysign(max_acc_change_dt, acc_change_x);
    }
    if (std::abs(acc_change_y) > max_acc_change_dt) {
        acc_change_y = std::copysign(max_acc_change_dt, acc_change_y);
    }
    
    x_pred.data[int(OdomState::VEL_X)] = vel_x + acc_change_x;
    x_pred.data[int(OdomState::VEL_Y)] = vel_y + acc_change_y;
    
    // Acceleration: Nonlinear decay model based on velocity
    // Higher velocities tend to have more drag/resistance
    float drag_factor = 1.0f - DRAG_COEFFICIENT * vel_magnitude;
    drag_factor = std::max(MIN_DRAG_FACTOR, std::min(MAX_DRAG_FACTOR, drag_factor));
    
    x_pred.data[int(OdomState::ACC_X)] = acc_x * drag_factor;
    x_pred.data[int(OdomState::ACC_Y)] = acc_y * drag_factor;
    
    // Bias: Random walk with temperature/aging effects
    // In a real implementation, this could incorporate temperature sensors from the IMU
    // TODO: Tune based on IMU expected drift following calibration
    x_pred.data[int(OdomState::BIAS_ACC_X)] = x_prev.data[int(OdomState::BIAS_ACC_X)] * IMU_BIAS_AGING_FACTOR;
    x_pred.data[int(OdomState::BIAS_ACC_Y)] = x_prev.data[int(OdomState::BIAS_ACC_Y)] * IMU_BIAS_AGING_FACTOR;
}

void FourWheelEKFOdometry::observationFunction(
    const ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::StateVector& x,
    ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::InputVector& h_x)
{
    // Nonlinear observation model for X-drive kinematics
    
    float chassis_vel_x = x.data[int(OdomState::VEL_X)];
    float chassis_vel_y = x.data[int(OdomState::VEL_Y)];
    float vel_magnitude = std::sqrt(chassis_vel_x * chassis_vel_x + chassis_vel_y * chassis_vel_y);
    
    // Nonlinear scaling factor based on velocity magnitude
    // At low speeds, wheel slippage becomes more significant
    float slip_factor = 1.0f;
    if (vel_magnitude > SLIP_VELOCITY_THRESHOLD) {
        slip_factor = 1.0f + SLIP_SCALING_FACTOR * std::tanh(vel_magnitude - SLIP_TANH_OFFSET);
    } else {
        slip_factor = LOW_VELOCITY_SLIP_FACTOR;
    }
    
    // X-drive wheel velocity mapping with nonlinear corrections using wheel configurations
    // Each wheel sees the chassis velocity transformed by its orientation and position
    for (int i = 0; i < 4; i++) {
        float wheel_angle = WHEEL_CONFIGS[i].wheelOrientationToForwardRadians;
        float cos_angle = std::cos(wheel_angle);
        float sin_angle = std::sin(wheel_angle);
        
        // Apply per-wheel slip and friction coefficients
        float wheel_slip_factor = slip_factor * WHEEL_CONFIGS[i].slipCoefficient;
        float wheel_friction_factor = WHEEL_CONFIGS[i].frictionCoefficient;
        
        // Transform chassis velocity to wheel velocity components
        h_x.data[int(OdomInput::VEL_X_1) + i * 2] = (chassis_vel_x * cos_angle + chassis_vel_y * sin_angle) * wheel_slip_factor * wheel_friction_factor;
        h_x.data[int(OdomInput::VEL_Y_1) + i * 2] = (-chassis_vel_x * sin_angle + chassis_vel_y * cos_angle) * wheel_slip_factor * wheel_friction_factor;
    }
    
    // IMU acceleration measurements with nonlinear bias correction
    // Bias correction depends on acceleration magnitude (sensor nonlinearity)
    float acc_x = x.data[int(OdomState::ACC_X)];
    float acc_y = x.data[int(OdomState::ACC_Y)];
    float bias_x = x.data[int(OdomState::BIAS_ACC_X)];
    float bias_y = x.data[int(OdomState::BIAS_ACC_Y)];
    
    float acc_magnitude = std::sqrt(acc_x * acc_x + acc_y * acc_y);
    float bias_scaling = 1.0f + BIAS_ACCELERATION_SCALING * acc_magnitude;
    
    h_x.data[int(OdomInput::ACC_X)] = acc_x + bias_x * bias_scaling;
    h_x.data[int(OdomInput::ACC_Y)] = acc_y + bias_y * bias_scaling;
}

void FourWheelEKFOdometry::stateJacobianFunction(
    const ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::StateVector& x,
    ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::StateMatrix& F,
    float dt)
{
    // Jacobian of nonlinear state transition function: F = ∂f/∂x
    
    // Initialize to zero
    for (int i = 0; i < int(OdomState::NUM_STATES) * int(OdomState::NUM_STATES); i++)
    {
        F.data[i] = 0.0f;
    }
    
    float vel_x = x.data[int(OdomState::VEL_X)];
    float vel_y = x.data[int(OdomState::VEL_Y)];
    float vel_magnitude = std::sqrt(vel_x * vel_x + vel_y * vel_y);
    float vel_correction_factor = 1.0f + VELOCITY_CORRECTION_COEFFICIENT * vel_magnitude;
    
    // Position derivatives (now velocity-dependent due to correction factor)
    F.data[int(OdomState::POS_X) * int(OdomState::NUM_STATES) + int(OdomState::POS_X)] = 1.0f;
    F.data[int(OdomState::POS_X) * int(OdomState::NUM_STATES) + int(OdomState::VEL_X)] = dt * vel_correction_factor;
    F.data[int(OdomState::POS_X) * int(OdomState::NUM_STATES) + int(OdomState::ACC_X)] = 0.5f * dt * dt;
    
    // Nonlinear correction term: ∂(vel_x * dt * correction)/∂vel_x and ∂/∂vel_y
    if (vel_magnitude > 1e-6f) {
        float correction_derivative_x = dt * (VELOCITY_CORRECTION_COEFFICIENT * vel_x / vel_magnitude);
        float correction_derivative_y = dt * (VELOCITY_CORRECTION_COEFFICIENT * vel_y / vel_magnitude);
        F.data[int(OdomState::POS_X) * int(OdomState::NUM_STATES) + int(OdomState::VEL_X)] += correction_derivative_x * vel_x;
        F.data[int(OdomState::POS_X) * int(OdomState::NUM_STATES) + int(OdomState::VEL_Y)] = correction_derivative_y * vel_x;
    }
    
    F.data[int(OdomState::POS_Y) * int(OdomState::NUM_STATES) + int(OdomState::POS_Y)] = 1.0f;
    F.data[int(OdomState::POS_Y) * int(OdomState::NUM_STATES) + int(OdomState::VEL_Y)] = dt * vel_correction_factor;
    F.data[int(OdomState::POS_Y) * int(OdomState::NUM_STATES) + int(OdomState::ACC_Y)] = 0.5f * dt * dt;
    
    if (vel_magnitude > 1e-6f) {
        float correction_derivative_x = dt * (VELOCITY_CORRECTION_COEFFICIENT * vel_x / vel_magnitude);
        float correction_derivative_y = dt * (VELOCITY_CORRECTION_COEFFICIENT * vel_y / vel_magnitude);
        F.data[int(OdomState::POS_Y) * int(OdomState::NUM_STATES) + int(OdomState::VEL_X)] = correction_derivative_x * vel_y;
        F.data[int(OdomState::POS_Y) * int(OdomState::NUM_STATES) + int(OdomState::VEL_Y)] += correction_derivative_y * vel_y;
    }
    
    // Velocity derivatives (with acceleration limiting)
    F.data[int(OdomState::VEL_X) * int(OdomState::NUM_STATES) + int(OdomState::VEL_X)] = 1.0f;
    F.data[int(OdomState::VEL_X) * int(OdomState::NUM_STATES) + int(OdomState::ACC_X)] = dt; // Simplified, could add limiting logic
    
    F.data[int(OdomState::VEL_Y) * int(OdomState::NUM_STATES) + int(OdomState::VEL_Y)] = 1.0f;
    F.data[int(OdomState::VEL_Y) * int(OdomState::NUM_STATES) + int(OdomState::ACC_Y)] = dt;
    
    // Acceleration derivatives (velocity-dependent drag)
    float drag_factor = 1.0f - DRAG_COEFFICIENT * vel_magnitude;
    drag_factor = std::max(MIN_DRAG_FACTOR, std::min(MAX_DRAG_FACTOR, drag_factor));
    
    F.data[int(OdomState::ACC_X) * int(OdomState::NUM_STATES) + int(OdomState::ACC_X)] = drag_factor;
    F.data[int(OdomState::ACC_Y) * int(OdomState::NUM_STATES) + int(OdomState::ACC_Y)] = drag_factor;
    
    // Drag derivatives with respect to velocity
    if (vel_magnitude > 1e-6f) {
        float drag_derivative_x = -DRAG_COEFFICIENT * vel_x / vel_magnitude;
        float drag_derivative_y = -DRAG_COEFFICIENT * vel_y / vel_magnitude;
        float acc_x = x.data[int(OdomState::ACC_X)];
        float acc_y = x.data[int(OdomState::ACC_Y)];
        
        F.data[int(OdomState::ACC_X) * int(OdomState::NUM_STATES) + int(OdomState::VEL_X)] = drag_derivative_x * acc_x;
        F.data[int(OdomState::ACC_X) * int(OdomState::NUM_STATES) + int(OdomState::VEL_Y)] = drag_derivative_y * acc_x;
        F.data[int(OdomState::ACC_Y) * int(OdomState::NUM_STATES) + int(OdomState::VEL_X)] = drag_derivative_x * acc_y;
        F.data[int(OdomState::ACC_Y) * int(OdomState::NUM_STATES) + int(OdomState::VEL_Y)] = drag_derivative_y * acc_y;
    }
    
    // Bias derivatives (aging factor)
    F.data[int(OdomState::BIAS_ACC_X) * int(OdomState::NUM_STATES) + int(OdomState::BIAS_ACC_X)] = IMU_BIAS_AGING_FACTOR;
    F.data[int(OdomState::BIAS_ACC_Y) * int(OdomState::NUM_STATES) + int(OdomState::BIAS_ACC_Y)] = IMU_BIAS_AGING_FACTOR;
}

void FourWheelEKFOdometry::observationJacobianFunction(
    const ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::StateVector& x,
    ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::ObservationMatrix& H)
{
    // Jacobian of nonlinear observation function: H = ∂h/∂x
    
    // Initialize to zero
    for (int i = 0; i < int(OdomInput::NUM_INPUTS) * int(OdomState::NUM_STATES); i++)
    {
        H.data[i] = 0.0f;
    }
    
    float chassis_vel_x = x.data[int(OdomState::VEL_X)];
    float chassis_vel_y = x.data[int(OdomState::VEL_Y)];
    float vel_magnitude = std::sqrt(chassis_vel_x * chassis_vel_x + chassis_vel_y * chassis_vel_y);
    
    // Slip factor and its derivatives
    float slip_factor = 1.0f;
    float slip_derivative_x = 0.0f, slip_derivative_y = 0.0f;
    
    if (vel_magnitude > SLIP_VELOCITY_THRESHOLD) {
        float tanh_arg = vel_magnitude - SLIP_TANH_OFFSET;
        float tanh_val = std::tanh(tanh_arg);
        float sech_squared = 1.0f - tanh_val * tanh_val;
        
        slip_factor = 1.0f + SLIP_SCALING_FACTOR * tanh_val;
        
        if (vel_magnitude > 1e-6f) {
            float slip_mag_derivative = SLIP_SCALING_FACTOR * sech_squared;
            slip_derivative_x = slip_mag_derivative * chassis_vel_x / vel_magnitude;
            slip_derivative_y = slip_mag_derivative * chassis_vel_y / vel_magnitude;
        }
    } else {
        slip_factor = LOW_VELOCITY_SLIP_FACTOR;
        // slip_derivatives remain 0 for low velocity region
    }
    
    // X-drive wheel velocity Jacobians using wheel configurations
    for (int i = 0; i < 4; i++) {
        float wheel_angle = WHEEL_CONFIGS[i].wheelOrientationToForwardRadians;
        float cos_angle = std::cos(wheel_angle);
        float sin_angle = std::sin(wheel_angle);
        
        // Per-wheel coefficients
        float wheel_slip_factor = slip_factor * WHEEL_CONFIGS[i].slipCoefficient;
        float wheel_friction_factor = WHEEL_CONFIGS[i].frictionCoefficient;
        float wheel_slip_derivative_x = slip_derivative_x * WHEEL_CONFIGS[i].slipCoefficient;
        float wheel_slip_derivative_y = slip_derivative_y * WHEEL_CONFIGS[i].slipCoefficient;
        
        // Jacobian for wheel X component: ∂(vx*cos + vy*sin)/∂[vx, vy]
        float base_coeff_x = cos_angle;
        float base_coeff_y = sin_angle;
        int vel_x_row = int(OdomInput::VEL_X_1) + i * 2;
        H.data[vel_x_row * int(OdomState::NUM_STATES) + int(OdomState::VEL_X)] = 
            (base_coeff_x * wheel_slip_factor + (chassis_vel_x * base_coeff_x + chassis_vel_y * base_coeff_y) * wheel_slip_derivative_x) * wheel_friction_factor;
        H.data[vel_x_row * int(OdomState::NUM_STATES) + int(OdomState::VEL_Y)] = 
            (base_coeff_y * wheel_slip_factor + (chassis_vel_x * base_coeff_x + chassis_vel_y * base_coeff_y) * wheel_slip_derivative_y) * wheel_friction_factor;
        
        // Jacobian for wheel Y component: ∂(-vx*sin + vy*cos)/∂[vx, vy]
        float base_coeff_y_comp_x = -sin_angle;
        float base_coeff_y_comp_y = cos_angle;
        int vel_y_row = int(OdomInput::VEL_Y_1) + i * 2;
        H.data[vel_y_row * int(OdomState::NUM_STATES) + int(OdomState::VEL_X)] = 
            (base_coeff_y_comp_x * wheel_slip_factor + (chassis_vel_x * base_coeff_y_comp_x + chassis_vel_y * base_coeff_y_comp_y) * wheel_slip_derivative_x) * wheel_friction_factor;
        H.data[vel_y_row * int(OdomState::NUM_STATES) + int(OdomState::VEL_Y)] = 
            (base_coeff_y_comp_y * wheel_slip_factor + (chassis_vel_x * base_coeff_y_comp_x + chassis_vel_y * base_coeff_y_comp_y) * wheel_slip_derivative_y) * wheel_friction_factor;
    }
    
    // IMU acceleration observations with nonlinear bias scaling
    float acc_x = x.data[int(OdomState::ACC_X)];
    float acc_y = x.data[int(OdomState::ACC_Y)];
    float bias_x = x.data[int(OdomState::BIAS_ACC_X)];
    float bias_y = x.data[int(OdomState::BIAS_ACC_Y)];
    
    float acc_magnitude = std::sqrt(acc_x * acc_x + acc_y * acc_y);
    float bias_scaling = 1.0f + BIAS_ACCELERATION_SCALING * acc_magnitude;
    
    // ∂(acc_x + bias_x * scaling)/∂acc_x
    H.data[int(OdomInput::ACC_X) * int(OdomState::NUM_STATES) + int(OdomState::ACC_X)] = 
        1.0f + bias_x * BIAS_ACCELERATION_SCALING * (acc_magnitude > 1e-6f ? acc_x / acc_magnitude : 0.0f);
    H.data[int(OdomInput::ACC_X) * int(OdomState::NUM_STATES) + int(OdomState::ACC_Y)] = 
        bias_x * BIAS_ACCELERATION_SCALING * (acc_magnitude > 1e-6f ? acc_y / acc_magnitude : 0.0f);
    H.data[int(OdomInput::ACC_X) * int(OdomState::NUM_STATES) + int(OdomState::BIAS_ACC_X)] = bias_scaling;
    
    // Similar for ACC_Y
    H.data[int(OdomInput::ACC_Y) * int(OdomState::NUM_STATES) + int(OdomState::ACC_X)] = 
        bias_y * BIAS_ACCELERATION_SCALING * (acc_magnitude > 1e-6f ? acc_x / acc_magnitude : 0.0f);
    H.data[int(OdomInput::ACC_Y) * int(OdomState::NUM_STATES) + int(OdomState::ACC_Y)] = 
        1.0f + bias_y * BIAS_ACCELERATION_SCALING * (acc_magnitude > 1e-6f ? acc_y / acc_magnitude : 0.0f);
    H.data[int(OdomInput::ACC_Y) * int(OdomState::NUM_STATES) + int(OdomState::BIAS_ACC_Y)] = bias_scaling;
}

}  // namespace aruwsrc::algorithms::odometry
