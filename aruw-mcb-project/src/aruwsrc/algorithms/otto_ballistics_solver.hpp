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

#ifndef OTTO_BALLISTICS_SOLVER_HPP_
#define OTTO_BALLISTICS_SOLVER_HPP_

#include <optional>

#include "tap/algorithms/transforms/transform.hpp"

// @todo genericize world frame
#include "aruwsrc/robot/sentry/sentry_transforms.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"

#include "otto_ballistics_solver.hpp"

#include "tap/algorithms/ballistics.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/odometry/odometry_2d_interface.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/launcher/launch_speed_predictor_interface.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/control/turret/robot_turret_subsystem.hpp"

#include "aruwsrc/robot/sentry/sentry_turret_minor_subsystem.hpp"
// #include "aruwsrc/robot/sentry/sentry_transform_constants.hpp"
// namespace aruwsrc::chassis
// {
// class HolonomicChassisSubsystem;
// }

// namespace aruwsrc::control::turret
// {
// class RobotTurretSubsystem;
// }

// namespace aruwsrc::control::launcher
// {
// class LaunchSpeedPredictorInterface;
// }

// namespace tap::algorithms::odometry
// {
// class Odometry2DInterface;
// }
namespace aruwsrc::algorithms
{
/**
 * An object that computes the world-relative pitch and yaw turret angles based on CV aim data and
 * odometry measurements.
 */
class OttoBallisticsSolver
{
public:
    // @todo make angles contiguous floats
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
     * Parameter to pass into `tap::algorithms::ballistics::findTargetProjectileIntersection`. This
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
                                         aruwsrc::algorithms::OttoBallisticsSolver::PLATE_WIDTH,
                                         2.0f * targetDistance)) &&
               (abs(pitchAngleError) < atan2f(
                                           aruwsrc::algorithms::OttoBallisticsSolver::PLATE_HEIGHT,
                                           2.0f * targetDistance));
    }

    /**
     * @param[in] odometryInterface Odometry object, used for position odometry information.
     * @param[in] turretSubsystem lol idk
     * @param[in] frictionWheels Friction wheels, used to determine the launch speed because leading
     * a target is a function of how fast a projectile is launched at.
     * @param[in] defaultLaunchSpeed The launch speed to be used in ballistics computation when the
     * friction wheels report the launch speed is 0 (i.e. when the friction wheels are off).
     */
    OttoBallisticsSolver(
        const tap::algorithms::odometry::Odometry2DInterface &odometryInterface,
        const aruwsrc::control::turret::SentryTurretMajorSubsystem &turretMajor,
        const aruwsrc::sentry::SentryTransforms &transforms,  // @todo only used for getting timestamp, which is bad
        const control::launcher::LaunchSpeedPredictorInterface &frictionWheels,
        const float defaultLaunchSpeed,
        const float turretToMajorRadius,
        uint8_t turretID);

    /**
     * Uses the `Odometry2DInterface` it has a pointer to, the chassis velocity, and the last aim
     * data to compute aim coordinates.
     *
     * This function verifies that the aim data it uses is valid (i.e.: it contains coords for a
     * real target and CV is online).
     *
     * @param[out] solution The ballistics solution computed. Will potentially update any of the
     * fields even if the solution's validSolutionFound function is false. Returns none if CV is
     * offline or trajectory intersection does not exist. 
     */
    mockable std::optional<BallisticsSolution> computeTurretAimAngles(const aruwsrc::serial::VisionCoprocessor::TurretAimData& aimData);

private:
    const tap::algorithms::odometry::Odometry2DInterface &odometryInterface;
    const aruwsrc::control::turret::SentryTurretMajorSubsystem &turretMajor;
    const control::launcher::LaunchSpeedPredictorInterface &frictionWheels;
    const float defaultLaunchSpeed;

    const aruwsrc::sentry::SentryTransforms &transforms;

    uint32_t lastAimDataTimestamp = 0;
    uint32_t lastOdometryTimestamp = 0;
    std::optional<BallisticsSolution> lastComputedSolution = {};

    uint8_t turretID;

public:
    // const uint8_t turretID;
    const float turretToMajorRadius;
}; // ottoballisticssolver

}  // namespace aruwsrc::algorithms

#endif  // OTTO_BALLISTICS_SOLVER_HPP_
