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

#ifndef THREE_DEADWHEEL_CHASSIS_KF_ODOMETRY_HPP_
#define THREE_DEADWHEEL_CHASSIS_KF_ODOMETRY_HPP_

#include "tap/algorithms/kalman_filter.hpp"
#include "tap/algorithms/odometry/chassis_displacement_observer_interface.hpp"
#include "tap/algorithms/odometry/chassis_world_yaw_observer_interface.hpp"
#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"

#include "aruwsrc/algorithms/odometry/otto_chassis_world_yaw_observer.hpp"
#include "aruwsrc/algorithms/odometry/three_deadwheel_odometry_observer.hpp"
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
class ThreeDeadwheelChassisKFOdometry : public tap::algorithms::odometry::Odometry2DInterface
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
     * @param parallelWheelChassisForwardRelativeAngleRadians Angle between the parallel deadwheel
     * and "forward" on the chassis
     * @param perpendicularWheelChassisForwardRelativeAngleRadians Angle between the perpendicular
     * deadwheel and "forward" on the chassis
     * @brief The parallel deadwheel is the deadwheel that is tangent to the edge of the chassis.
     * The perpendicular deadwheel is the deadwheel that is perpendicular to the edge of the
     * chassis. When moving in the direction of the parallel deadwheel, the perpendicular deadwheel
     * should not move, and vice versa
     */
    ThreeDeadwheelChassisKFOdometry(
        const aruwsrc::algorithms::odometry::ThreeDeadwheelOdometryObserver& deadwheelOdometry,
#if defined(TARGET_SENTRY_ECLIPSE)
        tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
#else
        aruwsrc::algorithms::odometry::OttoChassisWorldYawObserver& chassisYawObserver,
#endif
        tap::communication::sensors::imu::ImuInterface& imu,
        const modm::Vector2f initPos,
        const float initYaw,
        const float parallelOneCenterToWheelDistance,
        const float parallelTwoCenterToWheelDistance,
        const float perpendicularCenterToWheelDistance,
        const float parallelWheelOneChassisForwardRelativeAngleRadians,
        const float parallelWheelTwoChassisForwardRelativeAngleRadians,
        const float perpendicularWheelChassisForwardRelativeAngleRadians);

    inline modm::Location2D<float> getCurrentLocation2D() const final { return location; }

    inline modm::Vector2f getCurrentVelocity2D() const final { return velocity; }

    inline uint32_t getLastComputedOdometryTime() const final { return prevTime; }

    inline float getYaw() const override { return chassisYaw; }

    /**
     * @brief Resets the KF back to the robot's boot position.
     */
    void reset();

    void getOdometry();

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
        POS_ANG,
        VEL_ANG,
        NUM_STATES,
    };

    enum class OdomInput
    {
        VEL_X = 0,
        ACC_X,
        VEL_Y,
        ACC_Y,
        POS_ANG,
        VEL_ANG_ODOM,
        VEL_ANG_IMU,
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
        1, DT, 0.5 * DT * DT, 0, 0 , 0            , 0, 0 ,
        0, 1 , DT           , 0, 0 , 0            , 0, 0 ,
        0, 0 , 1            , 0, 0 , 0            , 0, 0 ,
        0, 0 , 0            , 1, DT, 0.5 * DT * DT, 0, 0 ,
        0, 0 , 0            , 0, 1 , DT           , 0, 0 ,
        0, 0 , 0            , 0, 0 , 1            , 0, 0 ,
        0, 0 , 0            , 0, 0 , 0            , 1, DT,
        0, 0 , 0            , 0, 0 , 0            , 0, 1 ,
    };
    static constexpr float KF_C[INPUTS_MULT_STATES] = {
        0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 1,
        0, 0, 0, 0, 0, 0, 0, 1,
    };

    static constexpr float KF_R[INPUTS_SQUARED] = {
        7.49565672e-05, 0, 0, 0, 0, 0, 0,
        0, 7.35872941e-04, 0, 0, 0, 0, 0,
        0, 0, 7.81982345e-05, 0, 0, 0, 0,
        0, 0, 0, 5.69132363e-04, 0, 0, 0,
        0, 0, 0, 0, 5.69132363e-04, 0, 0,
        0, 0, 0, 0, 0, 5.69132363e-04, 0,
        0, 0, 0, 0, 0, 0, 5.69132363e-04,
    }; //EG@TODO:

    static constexpr float KF_Q[STATES_SQUARED] = {
        2.276528e-15f, 2.276528e-12f, 1.138264e-09f, 0            , 0            , 0            , 0            , 0            ,
        2.276528e-12f, 2.276528e-09f, 1.138264e-06f, 0            , 0            , 0            , 0            , 0            ,
        1.138264e-09f, 1.138264e-06f, 5.69132e-04f , 0            , 0            , 0            , 0            , 0            ,
        0            , 0            , 0            , 2.276528e-15f, 2.276528e-12f, 1.138264e-09f, 0            , 0            ,
        0            , 0            , 0            , 2.276528e-12f, 2.276528e-09f, 1.138264e-06f, 0            , 0            ,
        0            , 0            , 0            , 1.138264e-09f, 1.138264e-06f, 5.69132e-04f , 0            , 0            ,
        0            , 0            , 0            , 0            , 0            , 0            , 2.276528e-15f, 2.276528e-12f,
        0            , 0            , 0            , 0            , 0            , 0            , 2.276528e-12f, 2.276528e-09f,
    }; //EG@TODO: Multiply by variance.
    
    static constexpr float KF_P0[STATES_SQUARED] = {
        1E-2, 0   , 0   , 0   , 0   , 0   , 0   , 0   ,
        0   , 1E-6, 0   , 0   , 0   , 0   , 0   , 0   ,
        0   , 0   , 1E+3, 0   , 0   , 0   , 0   , 0   ,
        0   , 0   , 0   , 1E-2, 0   , 0   , 0   , 0   ,
        0   , 0   , 0   , 0   , 1E-2, 0   , 0   , 0   ,
        0   , 0   , 0   , 0   , 0   , 1E-2, 0   , 0   ,
        0   , 0   , 0   , 0   , 0   , 0   , 1E-2, 0   ,
        0   , 0   , 0   , 0   , 0   , 0   , 0   , 1E-2,
    }; //EG@TODO:
    // clang-format on

    const aruwsrc::algorithms::odometry::ThreeDeadwheelOdometryObserver& deadwheelOdometry;
    tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver;
    tap::communication::sensors::imu::ImuInterface& imu;
    const modm::Vector2f initPos;
    const float initYaw;

    /// Chassis location in the world frame
    modm::Location2D<float> location;
    /// Chassis velocity in the world frame
    modm::Vector2f velocity;
    // Chassis yaw orientation in world frame (radians)
    float chassisYaw;

    float angularVelocity = 0;

    /// Previous time `update` was called, in microseconds
    uint32_t prevTime = 0;

    const float parallelOneCenterToWheelDistance;
    const float parallelTwoCenterToWheelDistance;
    const float perpendicularCenterToWheelDistance;
    const float parallelWheelOneChassisForwardRelativeAngleRadians;
    const float parallelWheelTwoChassisForwardRelativeAngleRadians;
    const float perpendicularWheelChassisForwardRelativeAngleRadians;
    void updateChassisStateFromKF();
    float perpendicularRaw;
    float parallelOneRaw;
    float parallelTwoRaw;
    float filteredPerpendicular;
    float filteredParallelOne;
    float filteredParallelTwo;
    float combinedParallel;

    static constexpr int FILTER_ORDER = 3;
    float parallelOneFilterState[FILTER_ORDER] = {0.0f};
    float parallelTwoFilterState[FILTER_ORDER] = {0.0f};
    float perpendicularFilterState[FILTER_ORDER] = {0.0f};
    float parallelNotchFilterState[3] = {0.0f};
    float perpendicularNotchFilterState[3] = {0.0f};

    static constexpr float IIR_A[FILTER_ORDER] = {1.000000f, -1.583541f, 0.656414f};
    static constexpr float IIR_B[FILTER_ORDER] = {0.018218f, 0.036436f, 0.018218f};

    float y[int(OdomInput::NUM_INPUTS)];
    float x[int(OdomState::NUM_STATES)];

    float applyIirFilter(float input, float* state, const float* a, const float* b, int order);
};
}  // namespace aruwsrc::algorithms::odometry

#endif  // THREE_DEADWHEEL_CHASSIS_KF_ODOMETRY_HPP_
