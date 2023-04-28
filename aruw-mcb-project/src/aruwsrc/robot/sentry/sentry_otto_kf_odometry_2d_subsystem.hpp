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

#ifndef SENTRY_OTTO_KF_ODOMETRY_2D_SUBSYSTEM_HPP_
#define SENTRY_OTTO_KF_ODOMETRY_2D_SUBSYSTEM_HPP_

#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/algorithms/odometry/odometry_2d_tracker.hpp"
#include "tap/control/subsystem.hpp"

#include "aruwsrc/algorithms/odometry/otto_chassis_world_yaw_observer.hpp"
#include "aruwsrc/algorithms/odometry/otto_kf_odometry_2d_subsystem.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "modm/math/geometry/location_2d.hpp"

// Forward declarations
namespace tap
{
class Drivers;
}
namespace aruwsrc::control::turret
{
class TurretSubsystem;
}

namespace tap::control::chassis
{
class ChassisSubsystemInterface;
}

namespace aruwsrc::algorithms::odometry
{
/**
 * @brief Kalman Filter-based odometry class for the Otto vision system on the sentry.
 *
 * User is responsible for registering this subsystem with the command scheduler, or using some
 * other mechanism to call the `refresh` function periodically.
 *
 * @see ChassisKFOdometry
 */
class SentryOttoKFOdometry2DSubsystem final : public tap::control::Subsystem, public tap::algorithms::odometry::Odometry2DInterface
{
public:
    /**
     * @param[in] drivers reference to tap drivers
     * @param[in] chassis const reference to aruwsrc SentryDriveSubsystem
     * @param[in] turret const reference to a TurretMotor object, @see OttoChassisWorldYawObserver
     * for how it is used
     */
    SentryOttoKFOdometry2DSubsystem(
        tap::Drivers& drivers,
        const aruwsrc::chassis::HolonomicChassisSubsystem& chassis,
        const aruwsrc::control::turret::TurretSubsystem& turret);

    void refresh() override;

    inline modm::Location2D<float> getCurrentLocation2D() const final { return location; }

    inline modm::Vector2f getCurrentVelocity2D() const final { return velocity; }

    inline uint32_t getLastComputedOdometryTime() const final { return prevTime; }

    inline float getYaw() const override { return chassisYaw; }

    void update();

private:
    /// Chassis location in the world frame
    modm::Location2D<float> location;
    /// Chassis velocity in the world frame
    modm::Vector2f velocity;
    /// Chassis yaw orientation in world frame (radians)
    float chassisYaw = 0;

    /// Previous time `update` was called, in microseconds
    uint32_t prevTime = 0;

    enum class OdomState
    {
        POS_Y = 0,
        VEL_Y,
        ACC_Y,
        NUM_STATES,
    };

    enum class OdomInput
    {
        POS_Y = 0,
        VEL_Y,
        ACC_Y,
        NUM_INPUTS,
    };

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
        1, DT, 0.5 * DT * DT,
        0, 1 , DT           ,
        0, 0 , 1            ,
    };
    static constexpr float KF_C[INPUTS_MULT_STATES] = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1,
    };
    static constexpr float KF_Q[STATES_SQUARED] = {
        1E0, 0  , 0  ,
        0  , 1E0, 0  ,
        0  , 0  , 1E0,
    };
    static constexpr float KF_R[INPUTS_SQUARED] = {
        1.0, 0  , 0  ,
        0  , 1.0, 0  ,
        0  , 0  , 1E6,
    };
    static constexpr float KF_P0[STATES_SQUARED] = {
        1E3, 0  , 0  ,
        0  , 1E3, 0  ,
        0  , 0  , 1E3,
    };
    // clang-format on

    const aruwsrc::chassis::HolonomicChassisSubsystem& chassis;
    aruwsrc::algorithms::odometry::OttoChassisWorldYawObserver chassisYawObserver;
    tap::communication::sensors::imu::ImuInterface& chassisIMU;

    tap::algorithms::KalmanFilter<
        static_cast<int>(OdomState::NUM_STATES),
        static_cast<int>(OdomInput::NUM_INPUTS)>
        kf;

    void updateChassisStateFromKF();
};

}  // namespace aruwsrc::algorithms::odometry

#endif  // SENTRY_OTTO_KF_ODOMETRY_2D_SUBSYSTEM_HPP_
