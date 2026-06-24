/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef CV_BALLISTICS_SOLVER_HPP_
#define CV_BALLISTICS_SOLVER_HPP_

#include <optional>

#include "tap/algorithms/transforms/transform.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/algorithms/odometry/transforms/transformer_interface.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"

namespace aruwsrc::communication::rtt
{
class RttTelemetry;
}

namespace aruwsrc::control::chassis
{
class HolonomicChassisSubsystem;
}

namespace aruwsrc::control::turret
{
class RobotTurretSubsystem;
}

namespace aruwsrc::communication::serial
{
class VisionCoprocessor;
}

namespace aruwsrc::control::launcher
{
class LaunchSpeedPredictorInterface;
}

namespace tap::algorithms::odometry
{
class Odometry2DInterface;
}

namespace aruwsrc::algorithms
{
enum class AimStrategy
{
    JITTER = 0,   // track the targeted plate and shoot whenever we can
    SHOT_GATING,  // track the center of the targeted robot and shoot whenever it will hit a plate
};

/**
 * An object that computes the world-relative pitch and yaw turret angles based on CV aim data and
 * odometry measurements.
 */
class CvBallisticsSolver
{
public:
    struct BallisticsSolution
    {
        /// The computed pitch angle in the world frame in radians.
        float pitchAngle;
        /// The computed yaw angle in the world frame in radians.
        float yawAngle;
        /// The computed yaw angular velocity in the world frame in radians.
        float yawVel;
        /// The computed yaw angular acceleration in the world frame in radians.
        float yawAcc;
        /// The computed straight line distance between the turret and target, in m.
        float distance;
        /// The expected time-of-flight until impact (in seconds).
        float timeOfFlight;
        /// The active plate index being targeted (0-3).
        uint8_t activePlateIndex;
        /// Whether shot gating is being used
        bool shotWindowValid = false;
        /// Center of the shot timing window (absolute timestamp in microseconds), valid when
        /// shotWindowValid is true. Represents the time at which shooting should hit the center
        /// of the targetted plate.
        uint64_t shotWindowCenter = 0;
        /// Half of the shot window width (microseconds), valid when shotWindowValid is true.
        uint64_t shotWindowHalfWidth = 0;
    };

    struct Config
    {
        // target robot angular velocity (rad/s) above which we switch to shot timing mode
        float shotTimingEntryThreshold;

        // target robot angular velocity (rad/s) below which switch to jitter aim mode
        float shotTimingExitThreshold;

        // default launch speed used if the friction wheels aren't spinning so that our ballistics
        // solution is still reasonable
        const float defaultLaunchSpeed;

        // signed distance between the turret pitch and yaw axes, with positive meaning the pitch
        // axis is ahead of yaw
        float turretPitchOffset;

        // minimum time after which a shot command is sent that the shot will actually fire
        float minimumShotDelay;

        // how far outside of the 90 degree region facing us that our currently targeted plate has
        // to be in order for jitter aim to select a new plate (radians)
        float jitterAimPlateReselectionAngularAllowance = 0.2f;
    };

    /**
     * Parameter to pass into `tap::algorithms::ballistics::findTargetProjectileIntersection`. This
     * function is an iterative ballistics solver, so this represents how many iterations to
     * perform the ballistics computation.
     */
    static constexpr float NUM_FORWARD_KINEMATIC_PROJECTIONS = 3;

    /// The width of a small armor plate, in m
    static constexpr float PLATE_WIDTH = 0.135f;
    /// The height of a small armor plate, in m
    static constexpr float PLATE_HEIGHT = 0.125f;

    /**
     * @return true if the specified yaw and pitch angle errors are small enough such that if a
     * projectile were to be launched, the projectile would hit a small armor plate at
     * targetDistance m away.
     */
    static inline bool withinAimingTolerance(
        float yawAngleError,
        float pitchAngleError,
        float targetDistance)
    {
        if (targetDistance < 0)
        {
            return false;
        }

        return (abs(yawAngleError) < atan2f(
                                         aruwsrc::algorithms::CvBallisticsSolver::PLATE_WIDTH,
                                         2.0f * targetDistance)) &&
               (abs(pitchAngleError) < atan2f(
                                           aruwsrc::algorithms::CvBallisticsSolver::PLATE_HEIGHT,
                                           2.0f * targetDistance));
    }

    /**
     * @param[in] visionCoprocessor Vision coprocessor for aim data.
     * @param[in] odometryInterface Odometry object, used for position odometry information.
     * @param[in] turretSubsystem Turret subsystem for offset information.
     * @param[in] frictionWheels Friction wheels, used to determine the launch speed because leading
     * a target is a function of how fast a projectile is launched at.
     * @param[in] defaultLaunchSpeed The launch speed to be used in ballistics computation when the
     * friction wheels report the launch speed is 0 (i.e. when the friction wheels are off).
     * @param[in] turretID The vision turret ID for whose ballistics trajectory we will be solving
     * for, see the VisionCoprocessor for more information about this id.
     * @param[in] minimumShotDelay Minimum time (seconds) we can react to a fire command in.
     * @param[in] telemetry Pointer to the RTT telemetry instance for logging (can be nullptr).
     */
    CvBallisticsSolver(
        const aruwsrc::communication::serial::VisionCoprocessor& visionCoprocessor,
        const aruwsrc::algorithms::odometry::transforms::TransformerInterface& transformer,
        const control::launcher::LaunchSpeedPredictorInterface& frictionWheels,
        const Config config,
        const uint8_t turretID,
        aruwsrc::communication::rtt::RttTelemetry* telemetry = nullptr);

    /**
     * Uses the `Odometry2DInterface` it has a pointer to, the chassis velocity, and the last aim
     * data to compute aim coordinates.
     *
     * This function verifies that the aim data it uses is valid (i.e.: it contains coords for a
     * real target and CV is online).
     *
     * @param[out] solution The ballistics solution computed. Will potentially update any of the
     * fields even if the solution's validSolutionFound function is false
     */
    mockable std::optional<BallisticsSolution> computeTurretAimAngles();

    inline uint8_t getTurretID() const { return turretID; }

    /**
     * @note Doesn't recalculate anything, use `computeTurretAimAngles` instead if that is desired.
     */
    inline std::optional<BallisticsSolution> getLastComputedSolution() const
    {
        return lastComputedSolution;
    }

private:
    const aruwsrc::communication::serial::VisionCoprocessor& visionCoprocessor;
    const aruwsrc::algorithms::odometry::transforms::TransformerInterface& transformer;
    const tap::algorithms::transforms::Transform& worldToTurret;
    const control::launcher::LaunchSpeedPredictorInterface& frictionWheels;
    const Config config;
    const uint8_t turretID;
    aruwsrc::communication::rtt::RttTelemetry* telemetry;

    AimStrategy aimStrategy;
    uint32_t lastAimDataTimestamp = 0;
    uint32_t lastOdometryTimestamp = 0;
    std::optional<BallisticsSolution> lastComputedSolution = {};

    /**
     * Selects plate and computes time window within which shots fired will hit the selected plate.
     */
    std::optional<BallisticsSolution> computePulseEstimation(
        const communication::serial::VisionCoprocessor::TargetState& targetData,
        float launchSpeed);

    /**
     * Computes jitter aim solution, attempting to aim at the best possible plate at any moment.
     */
    std::optional<BallisticsSolution> computeJitterAim(
        const communication::serial::VisionCoprocessor::TargetState& targetData,
        float launchSpeed);
};
}  // namespace aruwsrc::algorithms

#endif  // CV_BALLISTICS_SOLVER_HPP_
