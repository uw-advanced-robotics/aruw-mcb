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

#include <algorithm>
#include <iterator>

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
        tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
        tap::communication::sensors::imu::ImuInterface& imu,
        const modm::Vector2f initPos,
        const float initYaw,
        const float parallelOneCenterToWheelDistance,
        const float parallelTwoCenterToWheelDistance,
        const float perpendicularCenterToWheelDistance,
        const float odomFrameToRobotFrame);

    inline modm::Location2D<float> getCurrentLocation2D() const final { return location; }

    inline modm::Vector2f getCurrentVelocity2D() const final { return velocity; }

    inline uint32_t getLastComputedOdometryTime() const final { return prevTime; }

    inline float getYaw() const override { return chassisYaw.getWrappedValue(); }

    /**
     * @brief Resets the KF back to the robot's boot position.
     */
    void reset();

    void getOdometry();

    void update();

    void overrideOdometryPosition(const float positionX, const float positionY);
    void overrideOdometryOrientation(float yaw);

protected:
    enum class OdomStateX
    {
        POS_X = 0,
        VEL_X,
        NUM_STATES,
    };

    enum class XInput
    {
        VEL_X = 0,
        NUM_INPUTS,
    };

    enum class OdomStateY
    {
        POS_Y = 0,
        VEL_Y,
        NUM_STATES,
    };

    enum class YInput
    {
        VEL_Y = 0,
        NUM_INPUTS,
    };

    enum class OdomStateAng
    {
        POS_ANG = 0,
        VEL_ANG,
        NUM_STATES,
    };

    enum class AngInput
    {
        VEL_ANG_ODOM = 0,
        NUM_INPUTS,
    };

    using KF = tap::algorithms::KalmanFilter<int(OdomStateX::NUM_STATES), int(XInput::NUM_INPUTS)>;
    KF kf_x;
    KF kf_y;
    KF kf_ang;

private:
    static constexpr int X_STATES_SQUARED =
        static_cast<int>(OdomStateX::NUM_STATES) * static_cast<int>(OdomStateX::NUM_STATES);
    static constexpr int Y_STATES_SQUARED =
        static_cast<int>(OdomStateY::NUM_STATES) * static_cast<int>(OdomStateY::NUM_STATES);
    static constexpr int ANG_STATES_SQUARED =
        static_cast<int>(OdomStateAng::NUM_STATES) * static_cast<int>(OdomStateAng::NUM_STATES);
    static constexpr int X_INPUTS_SQUARED =
        static_cast<int>(XInput::NUM_INPUTS) * static_cast<int>(XInput::NUM_INPUTS);
    static constexpr int Y_INPUTS_SQUARED =
        static_cast<int>(YInput::NUM_INPUTS) * static_cast<int>(YInput::NUM_INPUTS);
    static constexpr int ANG_INPUTS_SQUARED =
        static_cast<int>(AngInput::NUM_INPUTS) * static_cast<int>(AngInput::NUM_INPUTS);
    static constexpr int X_INPUTS_MULT_STATES =
        static_cast<int>(XInput::NUM_INPUTS) * static_cast<int>(OdomStateX::NUM_STATES);
    static constexpr int Y_INPUTS_MULT_STATES =
        static_cast<int>(YInput::NUM_INPUTS) * static_cast<int>(OdomStateY::NUM_STATES);
    static constexpr int ANG_INPUTS_MULT_STATES =
        static_cast<int>(AngInput::NUM_INPUTS) * static_cast<int>(OdomStateAng::NUM_STATES);
    static_assert(X_STATES_SQUARED == Y_STATES_SQUARED);

    /// Assumed time difference between calls to `update`, in seconds
    static constexpr float DT = 0.002f;

    static constexpr float DT2 = DT * DT;
    static constexpr float DT3 = DT2 * DT;

    // clang-format off
    static constexpr float KF_A[X_STATES_SQUARED] = {
        1, DT, 
        0, 1 ,
    };

    static constexpr float KF_C[ANG_INPUTS_MULT_STATES] = {
        0, 1,
    };

    static constexpr float X_KF_R[X_INPUTS_SQUARED] = {
        1.67233E-07,
    };
    static constexpr float Y_KF_R[Y_INPUTS_SQUARED] = {
        7.96353E-07,
    };
    static constexpr float ANG_KF_R[ANG_INPUTS_SQUARED] = {
        2.00098E-05,
    };
    /// @TODO: TUNE

    static constexpr float JSD_X = 0.000871226;
    static constexpr float JSD_Y = 0.0001227612;
    static constexpr float JSD_THETA = 9.02143;

    static constexpr float X_KF_Q[X_STATES_SQUARED] = {
        JSD_X * DT3 / 3.0f, JSD_X * DT2 / 2.0f,
        JSD_X * DT2 / 2.0f, JSD_X * DT,
    };
    static constexpr float Y_KF_Q[Y_STATES_SQUARED] = {
        JSD_Y * DT3 / 3.0f, JSD_Y * DT2 / 2.0f,
        JSD_Y * DT2 / 2.0f, JSD_Y * DT,
    };
    static constexpr float ANG_KF_Q[ANG_STATES_SQUARED] = {
        JSD_THETA * DT3 / 3.0f, JSD_THETA * DT2 / 2.0f,
        JSD_THETA * DT2 / 2.0f, JSD_THETA * DT,
    };
    /// @TODO: TUNE

    static constexpr float X_KF_P0[X_STATES_SQUARED] = {
        1E-2, 0   ,
        0   , 1E-6,
    };
    static constexpr float Y_KF_P0[Y_STATES_SQUARED] = {
        1E-2, 0   , 
        0   , 1E-2,
    };
    static constexpr float ANG_KF_P0[ANG_STATES_SQUARED] = {
        1E-2, 0   ,
        0   , 1E-2,
    };
    /// @TODO: TUNE
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
    float angular_velocity;
    // Chassis yaw orientation in world frame (radians)
    tap::algorithms::Angle chassisYaw;

    /// Previous time `update` was called, in microseconds
    uint32_t prevTime = 0;

    tap::algorithms::Angle wrappedTheta = tap::algorithms::Angle(0.0f);
    tap::algorithms::Angle lastWrappedTheta = tap::algorithms::Angle(0.0f);
    tap::algorithms::Angle imuTheta = tap::algorithms::Angle(0.0f);

    const float parallelOneCenterToWheelDistance;
    const float parallelTwoCenterToWheelDistance;
    const float perpendicularCenterToWheelDistance;
    const float odomFrameToRobotFrame;
    void updateChassisStateFromKF();

    float x[int(OdomStateX::NUM_STATES)];
};
}  // namespace aruwsrc::algorithms::odometry

#endif  // THREE_DEADWHEEL_CHASSIS_KF_ODOMETRY_HPP_
