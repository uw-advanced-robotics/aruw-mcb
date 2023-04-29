/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SENTRY_KF_ODOMETRY_HPP_
#define SENTRY_KF_ODOMETRY_HPP_

#include "tap/algorithms/kalman_filter.hpp"
#include "tap/algorithms/odometry/chassis_displacement_observer_interface.hpp"
#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
// #include "tap/communication/sensors/imu/imu_interface.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"

#include "aruwsrc/algorithms/odometry/otto_chassis_world_yaw_observer.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/chassis/sentry/sentry_drive_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_turret_major_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_turret_minor_subsystem.hpp"
#include "modm/math/geometry/location_2d.hpp"
#include "modm/math/interpolation/linear.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"

#include "aruwsrc/communication/mcb-lite/virtual_imu_interface.hpp"
#include "tap/communication/sensors/imu/mpu6500/mpu6500.hpp"

namespace aruwsrc::algorithms::odometry
{
/**
 * An odometry interface that uses a kalman filter to measure odometry. This class is designed
 * specifically for the sentry.
 *
 * @note Assumes the world frame has an origin of (0, 0) wherever the robot was booted from.
 */
class SentryKFOdometry
{
public:
    /** TODO: update parameter documentation
     * @param[in] drivers: A reference to the drivers object. Used to obtain chassis IMU
     * measurements from the mpu6500 onboard the MCB, as well as to construct an
     * `OttoChassisWorldYawObserver` so that this class can be an `Odometry2DInterface`.
     * @param[in] chassis: A const reference to the sentry drive subsystem. Used to fetch
     * velocity and absolute position measurements from the encoders (and limit switches for the
     * latter).
     * @param[in] turret: A const reference to the turret subsystem from which odometry should be
     * tracked. In general, the only purpose of this is to produce yaw measurements, which for this
     * implementation does not affect the calculated kinematic state of the sentry chassis.
     * However, it is needed for inheritance since an `OttoChassisWorldYawObserver` is needed to
     * qualify this class as an `Odometry2DInterface`.
     */
    SentryKFOdometry(
        aruwsrc::serial::VisionCoprocessor& visionCoprocessor,
        aruwsrc::virtualMCB::VirtualIMUInterface chassisIMU,
        const aruwsrc::chassis::HolonomicChassisSubsystem& chassis,
        const aruwsrc::control::turret::SentryTurretMajorSubsystem& turretMajor,
        const aruwsrc::control::turret::SentryTurretMinorSubsystem& turretMinorLeft,
        const aruwsrc::control::turret::SentryTurretMinorSubsystem& turretMinorRight);

    void update();

    modm::Location2D<float> getCurrentLocation2D();

    modm::Vector2f getCurrentVelocity2D();

    uint32_t getLastComputedOdometryTime();

    /**
     * @return chassis yaw in world frame.
     */
    float getChassisYaw();

    /**
     * @return major yaw in world frame.
     */
    float getMajorYaw();

    /**
     * @return left minor yaw in world frame.
     */
    float getLeftMinorYaw();

    /**
     * @return left minor pitch in world frame.
     */
    float getLeftMinorPitch();

    /**
     * @return right minor yaw in world frame.
     */
    float getRightMinorYaw();

    /**
     * @return right minor pitch in world frame.
     */
    float getRightMinorPitch();

private:
    
    aruwsrc::serial::VisionCoprocessor& visionCoprocessor;
    aruwsrc::virtualMCB::VirtualIMUInterface chassisIMU;
    const aruwsrc::chassis::HolonomicChassisSubsystem& chassis;
    const aruwsrc::control::turret::SentryTurretMajorSubsystem& turretMajor;
    const aruwsrc::control::turret::SentryTurretMinorSubsystem& turretMinorLeft;
    const aruwsrc::control::turret::SentryTurretMinorSubsystem& turretMinorRight;

    /**
    * Resets the odometry using visioncoprocessor's localization method
    */
    // void resetOdometry(aruwsrc::serial::VisionCoprocessor::LocalizationCartesianData newData);

    static constexpr uint32_t UNINITIALIZED_TIMESTAMP = -1;

    uint32_t lastResetTimestamp = UNINITIALIZED_TIMESTAMP;

    float turretMajorYawError = 0.0f;

    float leftMinorYawError = 0.0f;
    float rightMinorYawError = 0.0f;

    // Kalman Filter enums
    // Chassis Inputs
    enum class OdomInput
    {
        VEL_X = 0,
        ACC_X,
        VEL_Y,
        ACC_Y,
        NUM_INPUTS
    };

    enum class OdomState
    {
        POS_X = 0,
        VEL_X,
        ACC_X,
        POS_Y,
        VEL_Y,
        ACC_Y,
        NUM_STATES
    };

    static constexpr int STATES_SQUARED =
        static_cast<int>(OdomState::NUM_STATES) * static_cast<int>(OdomState::NUM_STATES);
    static constexpr int INPUTS_SQUARED =
        static_cast<int>(OdomInput::NUM_INPUTS) * static_cast<int>(OdomInput::NUM_INPUTS);
    static constexpr int INPUTS_MULT_STATES =
        static_cast<int>(OdomInput::NUM_INPUTS) * static_cast<int>(OdomState::NUM_STATES);

    tap::algorithms::KalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)> kf;

    static constexpr float DT = 0.002f;

    // clang-format off

    /**
    * Positional Kalman Filter matrices
    */
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
        1E1, 0  , 0   , 0  , 0  , 0   ,
        0  , 1E0, 0   , 0  , 0  , 0   ,
        0  , 0  , 1E-1, 0  , 0  , 0   ,
        0  , 0  , 0   , 1E1, 0  , 0   ,
        0  , 0  , 0   , 0  , 1E0, 0   ,
        0  , 0  , 0   , 0  , 0  , 1E-1,
    };

    static constexpr float KF_R[INPUTS_SQUARED] = {
        1.0, 0  , 0  , 0  ,
        0  , 1.2, 0  , 0  ,
        0  , 0  , 1.0, 0  ,
        0  , 0  , 0  , 1.2,
    };

    static constexpr float KF_P0[STATES_SQUARED] = {
        1E3, 0  , 0  , 0  , 0  , 0  ,
        0  , 1E3, 0  , 0  , 0  , 0  ,
        0  , 0  , 1E3, 0  , 0  , 0  ,
        0  , 0  , 0  , 1E3, 0  , 0  ,
        0  , 0  , 0  , 0  , 1E3, 0  ,
        0  , 0  , 0  , 0  , 0  , 1E3,
    };
};
} // namespace namespace aruwsrc::algorithms::odometry

#endif // SENTRY_KF_ODOMETRY_HPP_
