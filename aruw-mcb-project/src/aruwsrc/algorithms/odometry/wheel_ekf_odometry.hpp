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

#ifndef WHEEL_EKF_ODOMETRY_HPP_
#define WHEEL_EKF_ODOMETRY_HPP_

#include "tap/algorithms/odometry/chassis_world_yaw_observer_interface.hpp"
#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"
#include "tap/motor/dji_motor.hpp"

#include "modm/math/geometry/location_2d.hpp"
#include "modm/math/geometry/vector.hpp"

#include "aruwsrc/algorithms/extended_kalman_filter.hpp"

namespace aruwsrc::chassis
{
class Holonomic4MotorChassisSubsystem;
}

namespace aruwsrc::algorithms::odometry
{
/**
 * An Extended Kalman Filter (EKF) based odometry interface that uses nonlinear motion models
 * for more accurate state prediction. This class is designed specifically for robots whose 
 * chassis does not measure absolute position (i.e. all ground robots).
 * 
 * The EKF provides better handling of nonlinear dynamics and can incorporate more complex
 * motion models compared to the linear Kalman filter.
 */
class FourWheelEKFOdometry : public tap::algorithms::odometry::Odometry2DInterface
{
public:
    struct ChassisWheelConfig {
        float wheelRadius;                          // Wheel radius in meters
        float wheelPositionX;                       // Wheel position relative to chassis center (m)
        float wheelPositionY;                       // Wheel position relative to chassis center (m)
        float wheelOrientationToForwardRadians;     // Wheel rolling direction relative to forward (rad)
        float motorToWheelGearRatio;                // Output/input gear ratio used for desired wheel speeds
    };

    /**
     * Constructor.
     *
     * @param chassisMotors The motors of the robot for odometry measurements
     * @param chassisYawObserver Interface that computes the yaw of the chassis externally
     * @param imu IMU mounted on the chassis to measure chassis acceleration
     * @param chassisSubsystem Optional chassis subsystem providing desired wheel outputs
     * @param initPos Initial position of chassis when robot boots
     */
    FourWheelEKFOdometry(
        const tap::motor::DjiMotor *chassisMotors[4],
        tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
        tap::communication::sensors::imu::ImuInterface& imu,
        const aruwsrc::chassis::Holonomic4MotorChassisSubsystem* chassisSubsystem,
        const modm::Vector2f initPos);

    inline modm::Location2D<float> getCurrentLocation2D() const final { return location; }

    inline modm::Vector2f getCurrentVelocity2D() const final { return velocity; }

    inline uint32_t getLastComputedOdometryTime() const final { return prevTime; }

    inline float getYaw() const override { return chassisYaw; }

    /**
     * @brief Resets the EKF back to the robot's boot position.
     */
    void reset();

    void update();

    void overrideOdometryPosition(const float positionX, const float positionY);

protected:
    enum class OdomState
    {
        POS_X = 0,      // World X position
        POS_Y,          // World Y position
        VEL_X,          // World X velocity
        VEL_Y,          // World Y velocity
        YAW,            // World yaw
        YAW_RATE,       // World yaw rate
        ACC_X,          // World X acceleration
        ACC_Y,          // World Y acceleration
        NUM_STATES,
    };

    enum class OdomInput
    {
        WHEEL_0 = 0,    // Wheel 0 linear speed (m/s)
        WHEEL_1,        // Wheel 1 linear speed (m/s)
        WHEEL_2,        // Wheel 2 linear speed (m/s)
        WHEEL_3,        // Wheel 3 linear speed (m/s)
        ACC_X,          // IMU acceleration X (world)
        ACC_Y,          // IMU acceleration Y (world)
        GYRO_Z,         // IMU yaw rate (rad/s)
        YAW,            // External yaw observation (rad)
        DESIRED_WHEEL_0,// Desired wheel 0 linear speed (m/s)
        DESIRED_WHEEL_1,// Desired wheel 1 linear speed (m/s)
        DESIRED_WHEEL_2,// Desired wheel 2 linear speed (m/s)
        DESIRED_WHEEL_3,// Desired wheel 3 linear speed (m/s)
        NUM_INPUTS,
    };

    ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)> ekf;

private:
    static constexpr int STATES_SQUARED =
        static_cast<int>(OdomState::NUM_STATES) * static_cast<int>(OdomState::NUM_STATES);
    static constexpr int INPUTS_SQUARED =
        static_cast<int>(OdomInput::NUM_INPUTS) * static_cast<int>(OdomInput::NUM_INPUTS);

    /// Assumed time difference between calls to `update`, in seconds.
    static constexpr float DT = 0.002f;  // Nominal EKF update period
    static constexpr float MIN_DT = 0.0005f;  // Lower bound on dt for stability

    static constexpr float BASE_WHEEL_MEASUREMENT_VARIANCE = 1.0f;  // Base wheel speed variance
    static constexpr float DESIRED_WHEEL_MEASUREMENT_VARIANCE = 1.0e3f;  // Low trust in commands
    static constexpr float IMU_ACCEL_MEASUREMENT_VARIANCE = 1.2f;  // Accel noise variance
    static constexpr float IMU_GYRO_MEASUREMENT_VARIANCE = 0.05f;  // Gyro noise variance
    static constexpr float YAW_MEASUREMENT_VARIANCE = 0.02f;  // Yaw observer variance


    // Process noise covariance matrix (Q) - how much we trust the motion model.
    // State order: POS_X, POS_Y, VEL_X, VEL_Y, YAW, YAW_RATE, ACC_X, ACC_Y.
    // Larger values = less trust in model, more responsive to measurements
    static constexpr float EKF_Q[STATES_SQUARED] = {
        1E2, 0  , 0  , 0  , 0  , 0  , 0  , 0  ,
        0  , 1E2, 0  , 0  , 0  , 0  , 0  , 0  ,
        0  , 0  , 1E1, 0  , 0  , 0  , 0  , 0  ,
        0  , 0  , 0  , 1E1, 0  , 0  , 0  , 0  ,
        0  , 0  , 0  , 0  , 1E-2,0  , 0  , 0  ,
        0  , 0  , 0  , 0  , 0  , 1E-1,0  , 0  ,
        0  , 0  , 0  , 0  , 0  , 0  , 5E0, 0  ,
        0  , 0  , 0  , 0  , 0  , 0  , 0  , 5E0,
    };

    // Measurement noise covariance matrix (R).
    // Measurement order: WHEEL_0..WHEEL_3, ACC_X, ACC_Y, GYRO_Z, YAW, DESIRED_WHEEL_0..3.
    // Higher value means less trust
    static constexpr float EKF_R[INPUTS_SQUARED] = {
        1.0, 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  ,
        0  , 1.0, 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  ,
        0  , 0  , 1.0, 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  ,
        0  , 0  , 0  , 1.0, 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  ,
        0  , 0  , 0  , 0  , 1.2, 0  , 0  , 0  , 0  , 0  , 0  , 0  ,
        0  , 0  , 0  , 0  , 0  , 1.2, 0  , 0  , 0  , 0  , 0  , 0  ,
        0  , 0  , 0  , 0  , 0  , 0  , 0.05,0  , 0  , 0  , 0  , 0  ,
        0  , 0  , 0  , 0  , 0  , 0  , 0  , 0.02,0  , 0  , 0  , 0  ,
        0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 1.0e3,0  , 0  , 0  ,
        0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 1.0e3,0  , 0  ,
        0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 1.0e3,0  ,
        0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 1.0e3,
    };

    // Initial covariance matrix (P0) - uncertainty in initial state estimates
    // Should be larger than steady-state uncertainties to allow filter to learn quickly
    static constexpr float EKF_P0[STATES_SQUARED] = {
        1E3, 0  , 0  , 0  , 0  , 0  , 0  , 0  ,
        0  , 1E3, 0  , 0  , 0  , 0  , 0  , 0  ,
        0  , 0  , 1E3, 0  , 0  , 0  , 0  , 0  ,
        0  , 0  , 0  , 1E3, 0  , 0  , 0  , 0  ,
        0  , 0  , 0  , 0  , 1E2, 0  , 0  , 0  ,
        0  , 0  , 0  , 0  , 0  , 1E2, 0  , 0  ,
        0  , 0  , 0  , 0  , 0  , 0  , 1E3, 0  ,
        0  , 0  , 0  , 0  , 0  , 0  , 0  , 1E3,
    };

    const tap::motor::DjiMotor *chassisMotors[4];
    tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver;
    tap::communication::sensors::imu::ImuInterface& imu;
    const aruwsrc::chassis::Holonomic4MotorChassisSubsystem* chassisSubsystem;

    const modm::Vector2f initPos;

    /// Chassis location in the world frame
    modm::Location2D<float> location;
    /// Chassis velocity in the world frame
    modm::Vector2f velocity;
    // Chassis yaw orientation in world frame (radians)
    float chassisYaw = 0;

    /// Previous time `update` was called, in microseconds
    uint32_t prevTime = 0;
    float prevWheelSpeeds[4]{0, 0, 0, 0};
    bool prevWheelSpeedsValid = false;

    /// Measurement vector for the EKF
    float z[int(OdomInput::NUM_INPUTS)];

    void updateChassisStateFromEKF();

    void updateMeasurementCovariance(
        const float wheelSpeeds[4],
        const float desiredWheelSpeeds[4],
        const modm::Vector2f& imuAccelWorld,
        bool yawMeasurementValid,
        float dt);

    // EKF function definitions
    static void stateTransitionFunction(
        const ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::StateVector& x_prev,
        ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::StateVector& x_pred,
        float dt);

    static void observationFunction(
        const ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::StateVector& x,
        ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::InputVector& h_x);

    static void stateJacobianFunction(
        const ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::StateVector& x,
        ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::StateMatrix& F,
        float dt);

    static void observationJacobianFunction(
        const ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::StateVector& x,
        ExtendedKalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)>::ObservationMatrix& H);

    static const ChassisWheelConfig WHEEL_CONFIGS[4];
};

}  // namespace aruwsrc::algorithms::odometry

#endif  // WHEEL_EKF_ODOMETRY_HPP_
