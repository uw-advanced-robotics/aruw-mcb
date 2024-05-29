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

#ifndef SENTRY_BALLISTICS_SOLVER_HPP_
#define SENTRY_BALLISTICS_SOLVER_HPP_

#include <optional>

#include "tap/algorithms/transforms/transform.hpp"

#include "aruwsrc/algorithms/odometry/transformer_interface.hpp"
#include "aruwsrc/algorithms/otto_ballistics_solver.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/control/turret/turret_motor.hpp"
#include "aruwsrc/control/turret/yaw_turret_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_transforms.hpp"

namespace aruwsrc::chassis
{
class HolonomicChassisSubsystem;
}

namespace aruwsrc::serial
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

namespace aruwsrc::sentry
{
/**
 * An object that computes the world-relative pitch and yaw turret angles based on CV aim data and
 * odometry measurements.
 */
class SentryBallisticsSolver
{
public:
    struct BallisticsSolution
    {
        /// The computed straight line distance between the turret and target, in m.
        float pitchAngle;
        /// The computed yaw angle in the world frame in radians.
        float yawAngle;
        /// The computed pitch angle in the world frame in radians.
        float distance;
        /// The expected time-of-flight until impact (in seconds).
        float timeOfFlight;
    };

    /**
     * Parameter to pass into `tap::algorithms::ballistics::findTargetProjectileIntersection`.
     This
     * function is an iterative ballistics solver, so this represents how many iterations to
     * perform the ballistics computation.
     */
    static constexpr float NUM_FORWARD_KINEMATIC_PROJECTIONS = 3;

    /// The width of a small armor plate, in m
    static constexpr float PLATE_WIDTH = 0.1f;
    /// The height of a small armor plate, in m
    static constexpr float PLATE_HEIGHT = 0.1f;

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
                                         aruwsrc::sentry::SentryBallisticsSolver::PLATE_WIDTH,
                                         2.0f * targetDistance)) &&
               (abs(pitchAngleError) < atan2f(
                                           aruwsrc::sentry::SentryBallisticsSolver::PLATE_HEIGHT,
                                           2.0f * targetDistance));
    }

    /**
     * @param[in] visionCoprocessor for getting target data
     * @param[in] transformer Transformer for getting chassis and turret odometry
     * @param[in] frictionWheels Friction wheels, used to determine the launch speed because leading
     * a target is a function of how fast a projectile is launched at.
     * @param[in] defaultLaunchSpeed The launch speed to be used in ballistics computation when the
     * friction wheels report the launch speed is 0 (i.e. when the friction wheels are off).
     * @param[in] worldToTurretBaseTransform transform from the world to the point the turret
     * rotates around (ex: transform to turret major for the sentry). This is used to get the
     * velocity of the turret
     * @param[in] turretBaseMotor motor that rotates the base on which the turret is mounted (ex:
     * motor of turret major for the sentry)
     * @param[in] turretDistFromBase the absolute distance of the turret from the point it spins
     * around. Should be 0 for standard and hero and nonzero for the dual-turret sentry.
     * @param[in] turretID The vision turret ID for whose ballistics trajectory we will be solving
     * for, see the VisionCoprocessor for more information about this id.
     */
    SentryBallisticsSolver(
        const aruwsrc::serial::VisionCoprocessor &visionCoprocessor,
        const aruwsrc::sentry::SentryTransforms &transformer,
        const control::launcher::LaunchSpeedPredictorInterface &frictionWheels,
        const aruwsrc::control::turret::YawTurretSubsystem &turretMajor,
        float defaultLaunchSpeed,
        float turretPitchOffset,
        // const aruwsrc::control::turret::TurretMotor &turretBaseMotor,
        const float turretMinorOffsetFromMajor,
        const uint8_t turretID);

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

private:
    const aruwsrc::serial::VisionCoprocessor &visionCoprocessor;
    const aruwsrc::sentry::SentryTransforms &transformer;
    const control::launcher::LaunchSpeedPredictorInterface &frictionWheels;
    const aruwsrc::control::turret::YawTurretSubsystem &turretMajor;
    const float defaultLaunchSpeed;
    const float turretPitchOffset;
    // const tap::algorithms::transforms::Transform &worldToTurretBaseTransform;
    // const aruwsrc::control::turret::TurretMotor &turretBaseMotor;
    const float turretDistFromBase;

    uint32_t lastAimDataTimestamp = 0;
    uint32_t lastOdometryTimestamp = 0;
    std::optional<BallisticsSolution> lastComputedSolution = {};

public:
    const uint8_t turretID;
};
}  // namespace aruwsrc::sentry

#endif  // SENTRY_BALLISTICS_SOLVER_HPP_
