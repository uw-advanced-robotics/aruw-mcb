/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef DEADWHEEL_CHASSIS_KF_ODOMETRY_HPP_
#define DEADWHEEL_CHASSIS_KF_ODOMETRY_HPP_

#include <aruwsrc/control/turret/yaw_turret_subsystem.hpp>

#include "tap/algorithms/kalman_filter.hpp"
#include "tap/algorithms/odometry/chassis_displacement_observer_interface.hpp"
#include "tap/algorithms/odometry/chassis_world_yaw_observer_interface.hpp"
#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"

#include "modm/math/geometry/location_2d.hpp"
#include "modm/math/interpolation/linear.hpp"

#include "two_deadwheel_odometry_observer.hpp"

namespace aruwsrc::algorithms::odometry
{
/**
 * An odometry interface that uses a kalman filter to measure odometry. This class is designed
 * specifically for robots whose chassis does not measure absolute position (i.e. all ground
 * robots). For those robots that measure chassis position directly (sentry, for example), a
 * tweaked version of the kalman filter used in this implementation should be used.
 */
class DeadwheelChassisKFOdometry : public tap::algorithms::odometry::Odometry2DInterface
{
public:
    /**
     * Constructor.
     *
     * @param deadwheelOdometry The deadwheels of the robot for odometry measurements
     * @param chassisYawObserver Interface that computes the yaw of the chassis externally
     * @param imu IMU mounted on the chassis to measure chassis acceleration
     * @param initPos Initial position of chassis when robot boots
     * @param parallelCenterToWheelDistance Distance from the center of the chassis to the center of
     * the parallel deadwheel
     * @param parallelWheelChassisRelativeAngleDegrees Angle between the parallel deadwheel and
     * "forward" on the chassis
     * @param perpendicularWheelChassisRelativeAngleDegrees Angle between the perpendicular
     * deadwheel and "forward" on the chassis
     * @brief The parallel deadwheel is the deadwheel that is tangent to the edge of the chassis.
     * The perpendicular deadwheel is the deadwheel that is perpendicular to the edge of the
     * chassis. When moving in the direction of the parallel deadwheel, the perpendicular deadwheel
     * should not move, and vice versa
     */
    DeadwheelChassisKFOdometry(
        const aruwsrc::algorithms::odometry::TwoDeadwheelOdometryObserver& deadwheelOdometry,
        tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
        tap::communication::sensors::imu::ImuInterface& imu,
        const modm::Vector2f initPos,
        const float parallelCenterToWheelDistance,
        const float parallelWheelChassisRelativeAngleDegrees,
        const float perpendicularWheelChassisRelativeAngleDegrees);

    inline modm::Location2D<float> getCurrentLocation2D() const final { return location; }

    inline modm::Vector2f getCurrentVelocity2D() const final { return velocity; }

    inline uint32_t getLastComputedOdometryTime() const final { return prevTime; }

    inline float getYaw() const override { return chassisYaw; }

    /**
     * @brief Resets the KF back to the robot's boot position.
     */
    void reset();

    void update();

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
        VEL_X = 0,
        ACC_X,
        VEL_Y,
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
    static constexpr float KF_C[INPUTS_MULT_STATES] = {
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1,
    };
    static constexpr float KF_Q[STATES_SQUARED] = {
        1E0, 0  , 0  , 0  , 0  , 0  ,
        0  , 1E0, 0  , 0  , 0  , 0  ,
        0  , 0  , 5E0, 0  , 0  , 0  ,
        0  , 0  , 0  , 1E0, 0  , 0  ,
        0  , 0  , 0  , 0  , 1E0, 0  ,
        0  , 0  , 0  , 0  , 0  , 5E0,
    };
    static constexpr float KF_R[INPUTS_SQUARED] = {
        1.0, 0  , 0  , 0  ,
        0  , 1.2, 0  , 0  ,
        0  , 0  , 1.0, 0  ,
        0  , 0  , 0  , 1.2,
    };
    static constexpr float KF_P0[STATES_SQUARED] = {
        1E0, 0  , 0  , 0  , 0  , 0  ,
        0  , 1E0, 0  , 0  , 0  , 0  ,
        0  , 0  , 1E3, 0  , 0  , 0  ,
        0  , 0  , 0  , 1E0, 0  , 0  ,
        0  , 0  , 0  , 0  , 1E0, 0  ,
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

    static constexpr float CHASSIS_WHEEL_ACCELERATION_LOW_PASS_ALPHA = 0.001f;

    const aruwsrc::algorithms::odometry::TwoDeadwheelOdometryObserver& deadwheelOdometry;
    tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver;
    tap::communication::sensors::imu::ImuInterface& imu;

    const modm::Vector2f initPos;

    static constexpr size_t VEL_AVG_WINDOW = 15;

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

    const float parallelCenterToWheelDistance;
    const float parallelWheelChassisRelativeAngleRadians;
    const float perpendicularWheelChassisRelativeAngleRadians;
    void updateChassisStateFromKF(float chassisYaw);

    void updateMeasurementCovariance(float Vx, float Vy);
};
}  // namespace aruwsrc::algorithms::odometry

#endif  // CHASSIS_KF_ODOMETRY_HPP_
