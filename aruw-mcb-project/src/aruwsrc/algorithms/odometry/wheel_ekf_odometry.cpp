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

#include "aruwsrc/communication/rtt/rtt_telemetry.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"

namespace aruwsrc::algorithms::odometry
{
const FourWheelEKFOdometry::ChassisWheelConfig FourWheelEKFOdometry::WHEEL_CONFIGS[4] = {
    // Wheel order matches sentryChassisMotorsForEkf: left front, right front, left back, right
    // back.
    {
        aruwsrc::control::chassis::WHEEL_RADIUS * WHEEL_RADIUS_SCALE,
        aruwsrc::control::chassis::WHEELBASE_RADIUS,
        aruwsrc::control::chassis::WHEELBASE_RADIUS,
        -M_PI_4,
    },
    {
        aruwsrc::control::chassis::WHEEL_RADIUS * WHEEL_RADIUS_SCALE,
        aruwsrc::control::chassis::WHEELBASE_RADIUS,
        -aruwsrc::control::chassis::WHEELBASE_RADIUS,
        -3.0f * M_PI_4,
    },
    {
        aruwsrc::control::chassis::WHEEL_RADIUS * WHEEL_RADIUS_SCALE,
        -aruwsrc::control::chassis::WHEELBASE_RADIUS,
        aruwsrc::control::chassis::WHEELBASE_RADIUS,
        M_PI_4,
    },
    {
        aruwsrc::control::chassis::WHEEL_RADIUS * WHEEL_RADIUS_SCALE,
        -aruwsrc::control::chassis::WHEELBASE_RADIUS,
        -aruwsrc::control::chassis::WHEELBASE_RADIUS,
        3.0f * M_PI_4,
    },
};

FourWheelEKFOdometry::FourWheelEKFOdometry(
    const tap::motor::DjiMotor* chassisMotors[4],
    tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
    tap::communication::sensors::imu::ImuInterface& imu,
    const modm::Vector2f initPos,
    aruwsrc::communication::rtt::RttTelemetry* telemetry)
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
      telemetry(telemetry)
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
    location = modm::Location2D<float>(initPos.x, initPos.y, 0.0f);
    velocity = modm::Vector2f(0.0f, 0.0f);
    chassisYaw = 0.0f;
    controlPredictedLocation = location;
    controlPredictedVelocity = velocity;
    controlPredictedYaw = chassisYaw;
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
    ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::InputVector
        measurement;

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
    bool wheelMotorOnline[4] = {false, false, false, false};
    uint8_t numOfflineWheels = 0;
    for (int i = 0; i < 4; i++)
    {
        wheelMotorOnline[i] = chassisMotors[i]->isMotorOnline();
        if (!wheelMotorOnline[i])
        {
            numOfflineWheels++;
        }

        float motorVel = wheelMotorOnline[i] ? chassisMotors[i]->getEncoder()->getVelocity() : 0.0f;
        wheelSpeeds[i] = motorVel * WHEEL_CONFIGS[i].wheelRadius;  // m/s
        measurement.data[int(OdomInput::WHEEL_0) + i] = wheelSpeeds[i];
    }

    // Get IMU acceleration data in chassis frame
    modm::Vector2f imuAccelWorld(imu.getAx(), imu.getAy());
    float yawForRotation = yawMeasurementValid ? measuredYaw : chassisYaw;
    if (yawOffsetInitialized)
    {
        yawForRotation = modm::Angle::normalize(yawForRotation - yawOffset);
    }

    // Rotate acceleration from MCB frame to the world frame.
    tap::algorithms::rotateVector(&imuAccelWorld.x, &imuAccelWorld.y, yawForRotation);

    measurement.data[int(OdomInput::ACC_X)] = imuAccelWorld.x;
    measurement.data[int(OdomInput::ACC_Y)] = imuAccelWorld.y;
    measurement.data[int(OdomInput::GYRO_Z)] = imu.getGz();
    measurement.data[int(OdomInput::YAW)] = yawForRotation;

    updateMeasurementCovariance(
        wheelSpeeds,
        wheelMotorOnline,
        imuAccelWorld,
        yawMeasurementValid,
        dt);

    // Perform prediction step.
    ekf.predict(dt);

    // Perform correction step with wrapped yaw residual to avoid discontinuities at +/-pi.
    ekf.updateWrapped(measurement, static_cast<uint16_t>(OdomInput::YAW));
    if (yawMeasurementValid)
    {
        auto& xState = ekf.getMutableStateVector();
        xState[int(OdomState::YAW)] = yawForRotation;
        xState[int(OdomState::YAW_RATE)] = measurement.data[int(OdomInput::GYRO_Z)];
    }

    // Update the location and velocity accessor objects with values from the state vector
    updateChassisStateFromEKF();
    captureControlPredictedState();
}

void FourWheelEKFOdometry::fuseVisionPosition(const VisionPositionMeasurement& measurement)
{
    fuseScalarMeasurement(
        OdomState::POS_X,
        measurement.position.x,
        measurement.positionVarianceX,
        false);
    fuseScalarMeasurement(
        OdomState::POS_Y,
        measurement.position.y,
        measurement.positionVarianceY,
        false);
    updateChassisStateFromEKF();
}

void FourWheelEKFOdometry::fuseVisionPose(const VisionPoseMeasurement& measurement)
{
    fuseVisionPosition(measurement);

    if (measurement.hasYaw)
    {
        float fusedYaw = measurement.yaw;
        if (yawOffsetInitialized)
        {
            fusedYaw = modm::Angle::normalize(fusedYaw - yawOffset);
        }
        fuseScalarMeasurement(OdomState::YAW, fusedYaw, measurement.yawVariance, true);
        updateChassisStateFromEKF();
    }
}

void FourWheelEKFOdometry::fuseLidarPosition(
    const modm::Vector2f& position,
    float positionVarianceX,
    float positionVarianceY)
{
    VisionPositionMeasurement measurement{
        position,
        positionVarianceX,
        positionVarianceY,
        VisionMeasurementSource::LIDAR};
    fuseVisionPosition(measurement);
}

void FourWheelEKFOdometry::initializeVisionPosition(
    const modm::Vector2f& position,
    float positionVarianceX,
    float positionVarianceY)
{
    auto& xState = ekf.getMutableStateVector();
    auto& covariance = ekf.getMutableStateCovariance();

    xState[int(OdomState::POS_X)] = position.x;
    xState[int(OdomState::POS_Y)] = position.y;

    covariance[int(OdomState::POS_X) * int(OdomState::NUM_STATES) + int(OdomState::POS_X)] =
        std::max(positionVarianceX, MIN_VISION_MEASUREMENT_VARIANCE);
    covariance[int(OdomState::POS_Y) * int(OdomState::NUM_STATES) + int(OdomState::POS_Y)] =
        std::max(positionVarianceY, MIN_VISION_MEASUREMENT_VARIANCE);

    updateChassisStateFromEKF();
    captureControlPredictedState();
}

void FourWheelEKFOdometry::initializeVisionPose(
    const modm::Vector2f& position,
    float yaw,
    float positionVarianceX,
    float positionVarianceY,
    float yawVariance)
{
    initializeVisionPosition(position, positionVarianceX, positionVarianceY);

    auto& xState = ekf.getMutableStateVector();
    auto& covariance = ekf.getMutableStateCovariance();

    float fusedYaw = yaw;
    if (yawOffsetInitialized)
    {
        fusedYaw = modm::Angle::normalize(fusedYaw - yawOffset);
    }

    xState[int(OdomState::YAW)] = fusedYaw;
    covariance[int(OdomState::YAW) * int(OdomState::NUM_STATES) + int(OdomState::YAW)] =
        std::max(yawVariance, MIN_VISION_MEASUREMENT_VARIANCE);

    updateChassisStateFromEKF();
    captureControlPredictedState();
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

void FourWheelEKFOdometry::captureControlPredictedState()
{
    controlPredictedLocation = location;
    controlPredictedVelocity = velocity;
    controlPredictedYaw = chassisYaw;
}

void FourWheelEKFOdometry::updateMeasurementCovariance(
    const float wheelSpeeds[4],
    const bool wheelMotorOnline[4],
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

    const float imuAccelMagnitude = imuAccelWorld.getLength();
    const float slipIndicator = std::max(0.0f, wheelAccelIndicator - imuAccelMagnitude);
    const float slipScale =
        1.0f + std::min(slipIndicator * WHEEL_SLIP_VARIANCE_SCALE, MAX_WHEEL_SLIP_SCALE - 1.0f);
    uint8_t numOfflineWheels = 0;
    for (int i = 0; i < 4; i++)
    {
        numOfflineWheels += !wheelMotorOnline[i];
    }

    const float onlineWheelVarianceScale =
        1.0f + numOfflineWheels * PARTIAL_WHEEL_OFFLINE_VARIANCE_SCALE;

    for (int i = 0; i < 4; i++)
    {
        float wheelVariance = wheelMotorOnline[i] ? BASE_WHEEL_MEASUREMENT_VARIANCE * slipScale *
                                                        onlineWheelVarianceScale
                                                  : OFFLINE_WHEEL_MEASUREMENT_VARIANCE;
        int wheelIndex = int(OdomInput::WHEEL_0) + i;
        R[wheelIndex * int(OdomInput::NUM_INPUTS) + wheelIndex] = wheelVariance;
    }

    int accXIndex = int(OdomInput::ACC_X);
    int accYIndex = int(OdomInput::ACC_Y);
    int gyroIndex = int(OdomInput::GYRO_Z);
    int yawIndex = int(OdomInput::YAW);
    const float accelMeasurementVariance =
        numOfflineWheels == 0 ? IMU_ACCEL_MEASUREMENT_VARIANCE : OFFLINE_ACCEL_MEASUREMENT_VARIANCE;
    R[accXIndex * int(OdomInput::NUM_INPUTS) + accXIndex] = accelMeasurementVariance;
    R[accYIndex * int(OdomInput::NUM_INPUTS) + accYIndex] = accelMeasurementVariance;
    R[gyroIndex * int(OdomInput::NUM_INPUTS) + gyroIndex] = IMU_GYRO_MEASUREMENT_VARIANCE;
    R[yawIndex * int(OdomInput::NUM_INPUTS) + yawIndex] =
        yawMeasurementValid ? YAW_MEASUREMENT_VARIANCE : 1.0e6f;

    if (numOfflineWheels == 0)
    {
        for (int i = 0; i < 4; i++)
        {
            prevWheelSpeeds[i] = wheelSpeeds[i];
        }
        prevWheelSpeedsValid = true;
    }
    else
    {
        prevWheelSpeedsValid = false;
    }
}

void FourWheelEKFOdometry::fuseScalarMeasurement(
    OdomState state,
    float measurement,
    float variance,
    bool wrapResidual)
{
    const int stateIndex = int(state);
    auto& x = ekf.getMutableStateVector();
    auto& P = ekf.getMutableStateCovariance();

    variance = std::max(variance, MIN_VISION_MEASUREMENT_VARIANCE);

    float innovation = measurement - x[stateIndex];
    if (wrapResidual)
    {
        innovation = std::atan2(std::sin(innovation), std::cos(innovation));
    }

    const float innovationCovariance =
        P[stateIndex * int(OdomState::NUM_STATES) + stateIndex] + variance;
    if (innovationCovariance <= MIN_VISION_MEASUREMENT_VARIANCE)
    {
        return;
    }

    float kalmanGain[int(OdomState::NUM_STATES)] = {0.0f};
    float measurementRow[int(OdomState::NUM_STATES)] = {0.0f};

    for (int i = 0; i < int(OdomState::NUM_STATES); i++)
    {
        kalmanGain[i] = P[i * int(OdomState::NUM_STATES) + stateIndex] / innovationCovariance;
        measurementRow[i] = P[stateIndex * int(OdomState::NUM_STATES) + i];
    }

    for (int i = 0; i < int(OdomState::NUM_STATES); i++)
    {
        x[i] += kalmanGain[i] * innovation;
    }

    for (int row = 0; row < int(OdomState::NUM_STATES); row++)
    {
        for (int col = 0; col < int(OdomState::NUM_STATES); col++)
        {
            P[row * int(OdomState::NUM_STATES) + col] -= kalmanGain[row] * measurementRow[col];
        }
    }

    for (int row = 0; row < int(OdomState::NUM_STATES); row++)
    {
        for (int col = row + 1; col < int(OdomState::NUM_STATES); col++)
        {
            const float symmetrizedCovariance = 0.5f * (P[row * int(OdomState::NUM_STATES) + col] +
                                                        P[col * int(OdomState::NUM_STATES) + row]);
            P[row * int(OdomState::NUM_STATES) + col] = symmetrizedCovariance;
            P[col * int(OdomState::NUM_STATES) + row] = symmetrizedCovariance;
        }

        const int diagonalIndex = row * int(OdomState::NUM_STATES) + row;
        P[diagonalIndex] = std::max(P[diagonalIndex], MIN_VISION_MEASUREMENT_VARIANCE);
    }

    x[int(OdomState::YAW)] = modm::Angle::normalize(x[int(OdomState::YAW)]);
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
    h_x.data[int(OdomInput::WHEEL_0)] = (vel_x_chassis - vel_y_chassis) / M_SQRT2;
    h_x.data[int(OdomInput::WHEEL_1)] = (-vel_x_chassis - vel_y_chassis) / M_SQRT2;
    h_x.data[int(OdomInput::WHEEL_2)] = (vel_x_chassis + vel_y_chassis) / M_SQRT2;
    h_x.data[int(OdomInput::WHEEL_3)] = (-vel_x_chassis + vel_y_chassis) / M_SQRT2;

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

    H.data[int(OdomInput::WHEEL_0) * int(OdomState::NUM_STATES) + vxIndex] =
        d_wheel0_d_vx / M_SQRT2;
    H.data[int(OdomInput::WHEEL_0) * int(OdomState::NUM_STATES) + vyIndex] =
        d_wheel0_d_vy / M_SQRT2;
    H.data[int(OdomInput::WHEEL_0) * int(OdomState::NUM_STATES) + yawIndex] =
        d_wheel0_d_yaw / M_SQRT2;

    H.data[int(OdomInput::WHEEL_1) * int(OdomState::NUM_STATES) + vxIndex] =
        d_wheel1_d_vx / M_SQRT2;
    H.data[int(OdomInput::WHEEL_1) * int(OdomState::NUM_STATES) + vyIndex] =
        d_wheel1_d_vy / M_SQRT2;
    H.data[int(OdomInput::WHEEL_1) * int(OdomState::NUM_STATES) + yawIndex] =
        d_wheel1_d_yaw / M_SQRT2;

    H.data[int(OdomInput::WHEEL_2) * int(OdomState::NUM_STATES) + vxIndex] =
        d_wheel2_d_vx / M_SQRT2;
    H.data[int(OdomInput::WHEEL_2) * int(OdomState::NUM_STATES) + vyIndex] =
        d_wheel2_d_vy / M_SQRT2;
    H.data[int(OdomInput::WHEEL_2) * int(OdomState::NUM_STATES) + yawIndex] =
        d_wheel2_d_yaw / M_SQRT2;

    H.data[int(OdomInput::WHEEL_3) * int(OdomState::NUM_STATES) + vxIndex] =
        d_wheel3_d_vx / M_SQRT2;
    H.data[int(OdomInput::WHEEL_3) * int(OdomState::NUM_STATES) + vyIndex] =
        d_wheel3_d_vy / M_SQRT2;
    H.data[int(OdomInput::WHEEL_3) * int(OdomState::NUM_STATES) + yawIndex] =
        d_wheel3_d_yaw / M_SQRT2;

    H.data[int(OdomInput::ACC_X) * int(OdomState::NUM_STATES) + int(OdomState::ACC_X)] = 1.0f;
    H.data[int(OdomInput::ACC_Y) * int(OdomState::NUM_STATES) + int(OdomState::ACC_Y)] = 1.0f;

    H.data[int(OdomInput::GYRO_Z) * int(OdomState::NUM_STATES) + int(OdomState::YAW_RATE)] = 1.0f;

    H.data[int(OdomInput::YAW) * int(OdomState::NUM_STATES) + int(OdomState::YAW)] = 1.0f;
}

}  // namespace aruwsrc::algorithms::odometry
