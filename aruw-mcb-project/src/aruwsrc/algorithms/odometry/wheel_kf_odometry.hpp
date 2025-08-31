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

#ifndef WHEEL_KF_ODOMETRY_HPP_
#define WHEEL_KF_ODOMETRY_HPP_

#include "tap/algorithms/kalman_filter.hpp"
#include "tap/algorithms/odometry/chassis_displacement_observer_interface.hpp"
#include "tap/algorithms/odometry/chassis_world_yaw_observer_interface.hpp"
#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"
#include "tap/motor/dji_motor.hpp"

#include "modm/math/geometry/location_2d.hpp"
#include "modm/math/interpolation/linear.hpp"

namespace aruwsrc::algorithms::odometry
{
/**
 * An odometry interface that uses a kalman filter to measure odometry. This class is designed
 * specifically for robots whose chassis does not measure absolute position (i.e. all ground
 * robots). For those robots that measure chassis position directly (sentry, for example), a
 * tweaked version of the kalman filter used in this implementation should be used.
 */



class FourWheelKFOdometry : public tap::algorithms::odometry::Odometry2DInterface
{
public:
    struct ChassisWheelConfig {
        float wheelRadius;
        float wheelbaseDistance;
        float wheelOrientationToForwardRadians;
    };
    /**
     * Constructor.
     *
     * @param chassisMotors The motors of the robot for odometry measurements
     * @param chassisWheelConfigs Configuration of the wheels on the chassis
     * @param chassisYawObserver Interface that computes the yaw of the chassis externally
     * @param imu IMU mounted on the chassis to measure chassis acceleration
     * @param initPos Initial position of chassis when robot boots
     */
    FourWheelKFOdometry(
        const tap::motor::DjiMotor *chassisMotors[4],
        const ChassisWheelConfig *chassisWheelConfigs[4],
        tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
        tap::communication::sensors::imu::ImuInterface& imu,
        const modm::Vector2f initPos);

    inline modm::Location2D<float> getCurrentLocation2D() const final { return location; }

    inline modm::Vector2f getCurrentVelocity2D() const final { return velocity; }

    inline uint32_t getLastComputedOdometryTime() const final { return prevTime; }

    inline float getYaw() const override { return chassisYaw; }

    void reset();

    void update();

    void overrideOdometryPosition(const float positionX, const float positionY);

protected:

    
    enum class OdomState
    {
        POS_X = 0,
        VEL_X,
        ACC_X,
        POS_Y,
        VEL_Y,
        ACC_Y,
        NUM_STATES,
    };

    enum class OdomInput
    {
        VEL_X_1 = 0,
        VEL_Y_1,
        VEL_X_2,
        VEL_Y_2,
        VEL_X_3,
        VEL_Y_3,
        VEL_X_4,
        VEL_Y_4,
        ACC_X,
        ACC_Y,
        NUM_INPUTS,
    };

    

    tap::algorithms::KalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)> kf;

private:
    static constexpr int STATES_SQUARED =
        static_cast<int>(OdomState::NUM_STATES) * static_cast<int>(OdomState::NUM_STATES);
    static constexpr int INPUTS_SQUARED =
        static_cast<int>(OdomInput::NUM_INPUTS) * static_cast<int>(OdomInput::NUM_INPUTS);
    static constexpr int INPUTS_MULT_STATES =
        static_cast<int>(OdomInput::NUM_INPUTS) * static_cast<int>(OdomState::NUM_STATES);

    /// Assumed time difference between calls to `update`, in seconds
    static constexpr float DT = 0.002f;

    // clang-format off
    static constexpr float KF_A[STATES_SQUARED] = {
        1, DT, 0.5 * DT * DT, 0, 0 , 0            ,
        0, 1 , DT           , 0, 0 , 0            ,
        0, 0 , 1            , 0, 0 , 0            ,
        0, 0 , 0            , 1, DT, 0.5 * DT * DT,
        0, 0 , 0            , 0, 1 , DT           ,
        0, 0 , 0            , 0, 0 , 1            ,
    };
    // clang-format off
    // This C matrix implements X-drive kinematics for 4 omni wheels
    // Wheel order: LF, LB, RF, RB with orientations: +45°, -45°, -45°, +45°
    // Each wheel contributes both X and Y components based on cos/sin of wheel angle
    static constexpr float KF_C[INPUTS_MULT_STATES] = {
        0, 0.25, 0, 0, 0, 0,      // VEL_X_1 (LF X-component): contributes to chassis X
        0, 0, 0, 0, 0.25, 0,      // VEL_Y_1 (LF Y-component): contributes to chassis Y
        0, 0.25, 0, 0, 0, 0,      // VEL_X_2 (LB X-component): contributes to chassis X
        0, 0, 0, 0, 0.25, 0,      // VEL_Y_2 (LB Y-component): contributes to chassis Y
        0, 0.25, 0, 0, 0, 0,      // VEL_X_3 (RF X-component): contributes to chassis X
        0, 0, 0, 0, 0.25, 0,      // VEL_Y_3 (RF Y-component): contributes to chassis Y
        0, 0.25, 0, 0, 0, 0,      // VEL_X_4 (RB X-component): contributes to chassis X
        0, 0, 0, 0, 0.25, 0,      // VEL_Y_4 (RB Y-component): contributes to chassis Y
        0, 0, 1, 0, 0, 0,         // ACC_X maps to chassis acceleration X
        0, 0, 0, 0, 0, 1,         // ACC_Y maps to chassis acceleration Y
    };
    static constexpr float KF_Q[STATES_SQUARED] = {
        1E2, 0  , 0  , 0  , 0  , 0  ,
        0  , 1E1, 0  , 0  , 0  , 0  ,
        0  , 0  , 5E0, 0  , 0  , 0  ,
        0  , 0  , 0  , 1E2, 0  , 0  ,
        0  , 0  , 0  , 0  , 1E1, 0  ,
        0  , 0  , 0  , 0  , 0  , 5E0,
    };
    static constexpr float KF_R[INPUTS_SQUARED] = {
        1.0, 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  ,  // VEL_X_1
        0  , 1.0, 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  ,  // VEL_Y_1
        0  , 0  , 1.0, 0  , 0  , 0  , 0  , 0  , 0  , 0  ,  // VEL_X_2
        0  , 0  , 0  , 1.0, 0  , 0  , 0  , 0  , 0  , 0  ,  // VEL_Y_2
        0  , 0  , 0  , 0  , 1.0, 0  , 0  , 0  , 0  , 0  ,  // VEL_X_3
        0  , 0  , 0  , 0  , 0  , 1.0, 0  , 0  , 0  , 0  ,  // VEL_Y_3
        0  , 0  , 0  , 0  , 0  , 0  , 1.0, 0  , 0  , 0  ,  // VEL_X_4
        0  , 0  , 0  , 0  , 0  , 0  , 0  , 1.0, 0  , 0  ,  // VEL_Y_4
        0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 1.2, 0  ,  // ACC_X
        0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 1.2,  // ACC_Y
    };
    static constexpr float KF_P0[STATES_SQUARED] = {
        1E3, 0  , 0  , 0  , 0  , 0  ,
        0  , 1E3, 0  , 0  , 0  , 0  ,
        0  , 0  , 1E3, 0  , 0  , 0  ,
        0  , 0  , 0  , 1E3, 0  , 0  ,
        0  , 0  , 0  , 0  , 1E3, 0  ,
        0  , 0  , 0  , 0  , 0  , 1E3,
    };
    // clang-format on

    /// Max chassis acceleration magnitude measured on the standard when at 120W power mode, in
    /// m/s^2. Also works for hero since it has an acceleration on the same order of magnitude.
    static constexpr float MAX_ACCELERATION = 8.0f;

    static constexpr modm::Pair<float, float> CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT[] =
        {
            {0, 1E0},
            {MAX_ACCELERATION, 1E2},
        };

    static constexpr float CHASSIS_WHEEL_ACCELERATION_LOW_PASS_ALPHA = 0.01f;

    const tap::motor::DjiMotor *chassisMotors[4];
    const ChassisWheelConfig *chassisWheelConfigs[4];
    tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver;
    tap::communication::sensors::imu::ImuInterface& imu;

    const modm::Vector2f initPos;

    /// Chassis location in the world frame
    modm::Location2D<float> location;
    /// Chassis velocity in the world frame
    modm::Vector2f velocity;
    // Chassis yaw orientation in world frame (radians)
    float chassisYaw = 0;

    /// Chassis measured change in velocity since the last time `update` was called, in the chassis
    /// frame
    modm::Vector2f chassisMeasuredDeltaVelocity;

    modm::interpolation::Linear<modm::Pair<float, float>>
        chassisAccelerationToMeasurementCovarianceInterpolator;

    /// Previous time `update` was called, in microseconds
    uint32_t prevTime = 0;
    modm::Matrix<float, 3, 1> prevChassisVelocity;

    /// Measurement vector for the Kalman filter
    float y[int(OdomInput::NUM_INPUTS)];

    void updateChassisStateFromKF(float chassisYaw);

    void updateMeasurementCovariance(const modm::Matrix<float, 3, 1>& chassisVelocity);

    float motorVel = 0;

    // Motor velocities
    float leftFrontMotorVel = 0;
    float leftBackMotorVel = 0;
    float rightFrontMotorVel = 0;
    float rightBackMotorVel = 0;

    // Wheel linear velocities
    float wheelLinearVel = 0;
    float leftFrontWheelLinearVel = 0;
    float leftBackWheelLinearVel = 0;
    float rightFrontWheelLinearVel = 0;
    float rightBackWheelLinearVel = 0;

    // X component velocities
    float leftFrontMotorLinearVelX = 0;
    float leftBackMotorLinearVelX = 0;
    float rightFrontMotorLinearVelX = 0;
    float rightBackMotorLinearVelX = 0;

    // Y component velocities
    float leftFrontMotorLinearVelY = 0;
    float leftBackMotorLinearVelY = 0;
    float rightFrontMotorLinearVelY = 0;
    float rightBackMotorLinearVelY = 0;

};
}  // namespace aruwsrc::algorithms::odometry

#endif  // CHASSIS_KF_ODOMETRY_HPP_