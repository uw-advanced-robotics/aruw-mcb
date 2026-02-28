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

#include <algorithm>
#include <cmath>

#include "tap/algorithms/math_user_utils.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"

namespace aruwsrc::algorithms::odometry
{
const FourWheelEKFOdometry::ChassisWheelConfig FourWheelEKFOdometry::WHEEL_CONFIGS[4] = {
    {
        aruwsrc::control::chassis::WHEEL_RADIUS * WHEEL_RADIUS_SCALE,
        aruwsrc::control::chassis::WHEELBASE_RADIUS,
        aruwsrc::control::chassis::WHEELBASE_RADIUS,
        M_PI_4,
    },
    {
        aruwsrc::control::chassis::WHEEL_RADIUS * WHEEL_RADIUS_SCALE,
        -aruwsrc::control::chassis::WHEELBASE_RADIUS,
        aruwsrc::control::chassis::WHEELBASE_RADIUS,
        -M_PI_4,
    },
    {
        aruwsrc::control::chassis::WHEEL_RADIUS * WHEEL_RADIUS_SCALE,
        aruwsrc::control::chassis::WHEELBASE_RADIUS,
        -aruwsrc::control::chassis::WHEELBASE_RADIUS,
        3.0f * M_PI_4,
    },
    {
        aruwsrc::control::chassis::WHEEL_RADIUS * WHEEL_RADIUS_SCALE,
        -aruwsrc::control::chassis::WHEELBASE_RADIUS,
        -aruwsrc::control::chassis::WHEELBASE_RADIUS,
        -3.0f * M_PI_4,
    },
};

FourWheelEKFOdometry::FourWheelEKFOdometry(
    const tap::motor::DjiMotor* chassisMotors[4],
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
      initPos(initPos)
{
    // Copy motor pointers to member array
    for (int i = 0; i < 4; i++)
    {
        this->chassisMotors[i] = chassisMotors[i];
    }
    reset();
}

void FourWheelEKFOdometry::reset()
{
    float initialX[int(OdomState::NUM_STATES)] =
        {initPos.x, initPos.y, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    ekf.init(initialX);
    prevTime = 0;
    prevWheelSpeedsValid = false;
    yawOffsetInitialized = false;
    for (int i = 0; i < 4; i++)
    {
        prevWheelSpeeds[i] = 0.0f;
    }
}

void FourWheelEKFOdometry::update()
{
    float measuredYaw = 0.0f;
    bool yawMeasurementValid = chassisYawObserver.getChassisWorldYaw(&measuredYaw);
    if (yawMeasurementValid)
    {
        measuredYaw = modm::Angle::normalize(measuredYaw);
    }
    if (!yawOffsetInitialized && yawMeasurementValid)
    {
        yawOffset = measuredYaw;
        yawOffsetInitialized = true;
    }

    uint32_t currentTime = tap::arch::clock::getTimeMicroseconds();
    float dt = prevTime == 0 ? DT : (currentTime - prevTime) / 1e6f;
    dt = std::clamp(dt, MIN_DT, MAX_DT);
    prevTime = currentTime;

    // Get individual wheel velocities and convert to linear velocities
    float wheelSpeeds[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; i++)
    {
        float motorVel = chassisMotors[i]->getEncoder()->getVelocity();  // rad/s (after gear ratio)
        wheelSpeeds[i] = motorVel * WHEEL_CONFIGS[i].wheelRadius;        // m/s
        z[int(OdomInput::WHEEL_0) + i] = wheelSpeeds[i];
    }

    // Get IMU acceleration data in chassis frame
    modm::Vector2f imuAccelWorld(imu.getAx(), imu.getAy());
    float yawForRotation = yawMeasurementValid ? measuredYaw : chassisYaw;
    if (yawOffsetInitialized)
    {
        yawForRotation = modm::Angle::normalize(yawForRotation - yawOffset);
    }

    // Rotate acceleration from MCB frame to the world frame.
    tap::algorithms::rotateVector(
        &imuAccelWorld.x,
        &imuAccelWorld.y,
        aruwsrc::communication::serial::VisionCoprocessor::MCB_ROTATION_OFFSET + yawForRotation);

    z[int(OdomInput::ACC_X)] = imuAccelWorld.x;
    z[int(OdomInput::ACC_Y)] = imuAccelWorld.y;
    z[int(OdomInput::GYRO_Z)] = imu.getGz();
    z[int(OdomInput::YAW)] = yawForRotation;

    updateMeasurementCovariance(wheelSpeeds, imuAccelWorld, yawMeasurementValid, dt);

    // Create measurement vector
    ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::InputVector
        measurement;
    for (int i = 0; i < int(OdomInput::NUM_INPUTS); i++)
    {
        measurement.data[i] = z[i];
    }

    // Perform prediction step.
    ekf.predict(dt);

    // Wrap yaw residual to avoid discontinuities at +/-pi.
    ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::InputVector
        z_pred;
    ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::StateVector
        x_current;
    const auto& stateArray = ekf.getStateVectorAsMatrix();
    for (int i = 0; i < int(OdomState::NUM_STATES); i++)
    {
        x_current.data[i] = stateArray[i];
    }
    observationFunction(x_current, z_pred);
    float yaw_residual = std::atan2(
        std::sin(measurement.data[int(OdomInput::YAW)] - z_pred.data[int(OdomInput::YAW)]),
        std::cos(measurement.data[int(OdomInput::YAW)] - z_pred.data[int(OdomInput::YAW)]));
    measurement.data[int(OdomInput::YAW)] = z_pred.data[int(OdomInput::YAW)] + yaw_residual;

    // Perform correction step.
    ekf.update(measurement);

    // Update the location and velocity accessor objects with values from the state vector
    updateChassisStateFromEKF();
}

void FourWheelEKFOdometry::updateChassisStateFromEKF()
{
    const auto& x = ekf.getStateVectorAsMatrix();

    // update odometry velocity and orientation
    velocity.x = x[int(OdomState::VEL_X)];
    velocity.y = x[int(OdomState::VEL_Y)];
    chassisYaw = modm::Angle::normalize(x[int(OdomState::YAW)]);

    // Set location
    location.setPosition(x[int(OdomState::POS_X)], x[int(OdomState::POS_Y)]);
    location.setOrientation(chassisYaw);
}

void FourWheelEKFOdometry::updateMeasurementCovariance(
    const float wheelSpeeds[4],
    const modm::Vector2f& imuAccelWorld,
    bool yawMeasurementValid,
    float dt)
{
    auto& R = ekf.getMeasurementCovariance();
    float wheelAccelIndicator = 0.0f;
    if (prevWheelSpeedsValid && dt > 1e-5f)
    {
        for (int i = 0; i < 4; i++)
        {
            float wheelAccel = (wheelSpeeds[i] - prevWheelSpeeds[i]) / dt;
            wheelAccelIndicator += std::abs(wheelAccel);
        }
        wheelAccelIndicator *= 0.25f;
    }

    float imuAccelMagnitude = imuAccelWorld.getLength();
    float slipIndicator = std::max(0.0f, wheelAccelIndicator - imuAccelMagnitude);
    float slipScale = 1.0f + slipIndicator;

    for (int i = 0; i < 4; i++)
    {
        float wheelVariance = BASE_WHEEL_MEASUREMENT_VARIANCE * slipScale;
        int wheelIndex = int(OdomInput::WHEEL_0) + i;
        R[wheelIndex * int(OdomInput::NUM_INPUTS) + wheelIndex] = wheelVariance;
    }

    int accXIndex = int(OdomInput::ACC_X);
    int accYIndex = int(OdomInput::ACC_Y);
    int gyroIndex = int(OdomInput::GYRO_Z);
    int yawIndex = int(OdomInput::YAW);
    R[accXIndex * int(OdomInput::NUM_INPUTS) + accXIndex] = IMU_ACCEL_MEASUREMENT_VARIANCE;
    R[accYIndex * int(OdomInput::NUM_INPUTS) + accYIndex] = IMU_ACCEL_MEASUREMENT_VARIANCE;
    R[gyroIndex * int(OdomInput::NUM_INPUTS) + gyroIndex] = IMU_GYRO_MEASUREMENT_VARIANCE;
    R[yawIndex * int(OdomInput::NUM_INPUTS) + yawIndex] =
        yawMeasurementValid ? YAW_MEASUREMENT_VARIANCE : 1.0e6f;

    for (int i = 0; i < 4; i++)
    {
        prevWheelSpeeds[i] = wheelSpeeds[i];
    }
    prevWheelSpeedsValid = true;
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
    const ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::StateVector&
        x_prev,
    ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::StateVector&
        x_pred,
    float dt)
{
    float pos_x = x_prev.data[int(OdomState::POS_X)];
    float pos_y = x_prev.data[int(OdomState::POS_Y)];
    float vel_x = x_prev.data[int(OdomState::VEL_X)];
    float vel_y = x_prev.data[int(OdomState::VEL_Y)];
    float yaw = x_prev.data[int(OdomState::YAW)];
    float yaw_rate = x_prev.data[int(OdomState::YAW_RATE)];
    float acc_x = x_prev.data[int(OdomState::ACC_X)];
    float acc_y = x_prev.data[int(OdomState::ACC_Y)];

    x_pred.data[int(OdomState::POS_X)] = pos_x + vel_x * dt + 0.5f * acc_x * dt * dt;
    x_pred.data[int(OdomState::POS_Y)] = pos_y + vel_y * dt + 0.5f * acc_y * dt * dt;
    x_pred.data[int(OdomState::VEL_X)] = vel_x + acc_x * dt;
    x_pred.data[int(OdomState::VEL_Y)] = vel_y + acc_y * dt;
    x_pred.data[int(OdomState::YAW)] = yaw + yaw_rate * dt;
    x_pred.data[int(OdomState::YAW_RATE)] = yaw_rate;
    x_pred.data[int(OdomState::ACC_X)] = acc_x;
    x_pred.data[int(OdomState::ACC_Y)] = acc_y;
}

void FourWheelEKFOdometry::observationFunction(
    const ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::StateVector&
        x,
    ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::InputVector& h_x)
{
    float vel_x_world = x.data[int(OdomState::VEL_X)];
    float vel_y_world = x.data[int(OdomState::VEL_Y)];
    float yaw = x.data[int(OdomState::YAW)];
    float yaw_rate = x.data[int(OdomState::YAW_RATE)];

    float cos_yaw = std::cos(yaw);
    float sin_yaw = std::sin(yaw);
    float vel_x_chassis = cos_yaw * vel_x_world + sin_yaw * vel_y_world;
    float vel_y_chassis = -sin_yaw * vel_x_world + cos_yaw * vel_y_world;
    const float rotationRadius = aruwsrc::control::chassis::WHEELBASE_RADIUS;
    h_x.data[int(OdomInput::WHEEL_0)] = vel_x_chassis - vel_y_chassis - rotationRadius * yaw_rate;
    h_x.data[int(OdomInput::WHEEL_1)] = -vel_x_chassis - vel_y_chassis - rotationRadius * yaw_rate;
    h_x.data[int(OdomInput::WHEEL_2)] = vel_x_chassis + vel_y_chassis - rotationRadius * yaw_rate;
    h_x.data[int(OdomInput::WHEEL_3)] = -vel_x_chassis + vel_y_chassis - rotationRadius * yaw_rate;

    float acc_x = x.data[int(OdomState::ACC_X)];
    float acc_y = x.data[int(OdomState::ACC_Y)];

    h_x.data[int(OdomInput::ACC_X)] = acc_x;
    h_x.data[int(OdomInput::ACC_Y)] = acc_y;
    h_x.data[int(OdomInput::GYRO_Z)] = yaw_rate;
    h_x.data[int(OdomInput::YAW)] = yaw;
}

void FourWheelEKFOdometry::stateJacobianFunction(
    const ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::StateVector&
        x,
    ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::StateMatrix& F,
    float dt)
{
    (void)x;
    // Initialize to zero
    for (int i = 0; i < int(OdomState::NUM_STATES) * int(OdomState::NUM_STATES); i++)
    {
        F.data[i] = 0.0f;
    }

    F.data[int(OdomState::POS_X) * int(OdomState::NUM_STATES) + int(OdomState::POS_X)] = 1.0f;
    F.data[int(OdomState::POS_X) * int(OdomState::NUM_STATES) + int(OdomState::VEL_X)] = dt;
    F.data[int(OdomState::POS_X) * int(OdomState::NUM_STATES) + int(OdomState::ACC_X)] =
        0.5f * dt * dt;

    F.data[int(OdomState::POS_Y) * int(OdomState::NUM_STATES) + int(OdomState::POS_Y)] = 1.0f;
    F.data[int(OdomState::POS_Y) * int(OdomState::NUM_STATES) + int(OdomState::VEL_Y)] = dt;
    F.data[int(OdomState::POS_Y) * int(OdomState::NUM_STATES) + int(OdomState::ACC_Y)] =
        0.5f * dt * dt;

    F.data[int(OdomState::VEL_X) * int(OdomState::NUM_STATES) + int(OdomState::VEL_X)] = 1.0f;
    F.data[int(OdomState::VEL_X) * int(OdomState::NUM_STATES) + int(OdomState::ACC_X)] = dt;

    F.data[int(OdomState::VEL_Y) * int(OdomState::NUM_STATES) + int(OdomState::VEL_Y)] = 1.0f;
    F.data[int(OdomState::VEL_Y) * int(OdomState::NUM_STATES) + int(OdomState::ACC_Y)] = dt;

    F.data[int(OdomState::YAW) * int(OdomState::NUM_STATES) + int(OdomState::YAW)] = 1.0f;
    F.data[int(OdomState::YAW) * int(OdomState::NUM_STATES) + int(OdomState::YAW_RATE)] = dt;

    F.data[int(OdomState::YAW_RATE) * int(OdomState::NUM_STATES) + int(OdomState::YAW_RATE)] = 1.0f;

    F.data[int(OdomState::ACC_X) * int(OdomState::NUM_STATES) + int(OdomState::ACC_X)] = 1.0f;
    F.data[int(OdomState::ACC_Y) * int(OdomState::NUM_STATES) + int(OdomState::ACC_Y)] = 1.0f;
}

void FourWheelEKFOdometry::observationJacobianFunction(
    const ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::StateVector&
        x,
    ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::ObservationMatrix&
        H)
{
    // Initialize to zero
    for (int i = 0; i < int(OdomInput::NUM_INPUTS) * int(OdomState::NUM_STATES); i++)
    {
        H.data[i] = 0.0f;
    }

    float vel_x_world = x.data[int(OdomState::VEL_X)];
    float vel_y_world = x.data[int(OdomState::VEL_Y)];
    float yaw = x.data[int(OdomState::YAW)];

    float cos_yaw = std::cos(yaw);
    float sin_yaw = std::sin(yaw);

    float d_vx_chassis_d_vx = cos_yaw;
    float d_vx_chassis_d_vy = sin_yaw;
    float d_vy_chassis_d_vx = -sin_yaw;
    float d_vy_chassis_d_vy = cos_yaw;
    float d_vx_chassis_d_yaw = -sin_yaw * vel_x_world + cos_yaw * vel_y_world;
    float d_vy_chassis_d_yaw = -cos_yaw * vel_x_world - sin_yaw * vel_y_world;

    const float rotationRadius = aruwsrc::control::chassis::WHEELBASE_RADIUS;
    const float d_wheel0_d_vx = d_vx_chassis_d_vx - d_vy_chassis_d_vx;
    const float d_wheel0_d_vy = d_vx_chassis_d_vy - d_vy_chassis_d_vy;
    const float d_wheel0_d_yaw = d_vx_chassis_d_yaw - d_vy_chassis_d_yaw;

    const float d_wheel1_d_vx = -d_vx_chassis_d_vx - d_vy_chassis_d_vx;
    const float d_wheel1_d_vy = -d_vx_chassis_d_vy - d_vy_chassis_d_vy;
    const float d_wheel1_d_yaw = -d_vx_chassis_d_yaw - d_vy_chassis_d_yaw;

    const float d_wheel2_d_vx = d_vx_chassis_d_vx + d_vy_chassis_d_vx;
    const float d_wheel2_d_vy = d_vx_chassis_d_vy + d_vy_chassis_d_vy;
    const float d_wheel2_d_yaw = d_vx_chassis_d_yaw + d_vy_chassis_d_yaw;

    const float d_wheel3_d_vx = -d_vx_chassis_d_vx + d_vy_chassis_d_vx;
    const float d_wheel3_d_vy = -d_vx_chassis_d_vy + d_vy_chassis_d_vy;
    const float d_wheel3_d_yaw = -d_vx_chassis_d_yaw + d_vy_chassis_d_yaw;

    const int vxIndex = int(OdomState::VEL_X);
    const int vyIndex = int(OdomState::VEL_Y);
    const int yawIndex = int(OdomState::YAW);
    const int yawRateIndex = int(OdomState::YAW_RATE);

    H.data[int(OdomInput::WHEEL_0) * int(OdomState::NUM_STATES) + vxIndex] = d_wheel0_d_vx;
    H.data[int(OdomInput::WHEEL_0) * int(OdomState::NUM_STATES) + vyIndex] = d_wheel0_d_vy;
    H.data[int(OdomInput::WHEEL_0) * int(OdomState::NUM_STATES) + yawIndex] = d_wheel0_d_yaw;
    H.data[int(OdomInput::WHEEL_0) * int(OdomState::NUM_STATES) + yawRateIndex] = -rotationRadius;

    H.data[int(OdomInput::WHEEL_1) * int(OdomState::NUM_STATES) + vxIndex] = d_wheel1_d_vx;
    H.data[int(OdomInput::WHEEL_1) * int(OdomState::NUM_STATES) + vyIndex] = d_wheel1_d_vy;
    H.data[int(OdomInput::WHEEL_1) * int(OdomState::NUM_STATES) + yawIndex] = d_wheel1_d_yaw;
    H.data[int(OdomInput::WHEEL_1) * int(OdomState::NUM_STATES) + yawRateIndex] = -rotationRadius;

    H.data[int(OdomInput::WHEEL_2) * int(OdomState::NUM_STATES) + vxIndex] = d_wheel2_d_vx;
    H.data[int(OdomInput::WHEEL_2) * int(OdomState::NUM_STATES) + vyIndex] = d_wheel2_d_vy;
    H.data[int(OdomInput::WHEEL_2) * int(OdomState::NUM_STATES) + yawIndex] = d_wheel2_d_yaw;
    H.data[int(OdomInput::WHEEL_2) * int(OdomState::NUM_STATES) + yawRateIndex] = -rotationRadius;

    H.data[int(OdomInput::WHEEL_3) * int(OdomState::NUM_STATES) + vxIndex] = d_wheel3_d_vx;
    H.data[int(OdomInput::WHEEL_3) * int(OdomState::NUM_STATES) + vyIndex] = d_wheel3_d_vy;
    H.data[int(OdomInput::WHEEL_3) * int(OdomState::NUM_STATES) + yawIndex] = d_wheel3_d_yaw;
    H.data[int(OdomInput::WHEEL_3) * int(OdomState::NUM_STATES) + yawRateIndex] = -rotationRadius;

    H.data[int(OdomInput::ACC_X) * int(OdomState::NUM_STATES) + int(OdomState::ACC_X)] = 1.0f;
    H.data[int(OdomInput::ACC_Y) * int(OdomState::NUM_STATES) + int(OdomState::ACC_Y)] = 1.0f;

    H.data[int(OdomInput::GYRO_Z) * int(OdomState::NUM_STATES) + int(OdomState::YAW_RATE)] = 1.0f;

    H.data[int(OdomInput::YAW) * int(OdomState::NUM_STATES) + int(OdomState::YAW)] = 1.0f;
}

}  // namespace aruwsrc::algorithms::odometry
