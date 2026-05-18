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

#include "sentry_ballistics_solver.hpp"

#include "tap/algorithms/ballistics.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/transforms/transform.hpp"
#include "tap/architecture/clock.hpp"

#include "aruwsrc/algorithms/spherical_projectile_aim.hpp"
#include "aruwsrc/communication/rtt/rtt_telemetry.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/launcher/launch_speed_predictor_interface.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
using namespace tap::algorithms;
using namespace modm;

namespace aruwsrc::sentry::algorithms
{
namespace
{
static constexpr uint8_t DRAG_FORWARD_KINEMATIC_PROJECTIONS = 1;
}  // namespace

SentryBallisticsSolver::SentryBallisticsSolver(
    const aruwsrc::communication::serial::VisionCoprocessor &visionCoprocessor,
    const odometry::SentryTransforms &transformer,
    const control::launcher::LaunchSpeedPredictorInterface &frictionWheels,
    const aruwsrc::control::turret::YawTurretSubsystem &turretMajor,
    const float defaultLaunchSpeed,
    const float turretPitchOffset,
    const float turretDistFromBase,
    const uint8_t turretID,
    aruwsrc::communication::rtt::RttTelemetry *,
    bool useDragCorrection)
    : visionCoprocessor(visionCoprocessor),
      transformer(transformer),
      frictionWheels(frictionWheels),
      turretMajor(turretMajor),
      defaultLaunchSpeed(defaultLaunchSpeed),
      turretPitchOffset(turretPitchOffset),
      turretDistFromBase(turretDistFromBase),
      useDragCorrection(useDragCorrection),
      turretID(turretID)
{
}

std::optional<SentryBallisticsSolver::BallisticsSolution> SentryBallisticsSolver::
    computeTurretAimAngles()
{
    const auto &aimData = visionCoprocessor.getLastAimData(turretID);
    // Verify that CV is actually online and that the aimData had a target
    if (!visionCoprocessor.isCvOnline() || !aimData.pva.updated)
    {
        lastComputedSolution = std::nullopt;
        return std::nullopt;
    }

    if (lastAimDataTimestamp != aimData.timestamp ||
        lastOdometryTimestamp != transformer.getLastComputedOdometryTime())
    {
        const uint32_t solveStartTime = tap::arch::clock::getTimeMicroseconds();

        lastAimDataTimestamp = aimData.timestamp;
        lastOdometryTimestamp = transformer.getLastComputedOdometryTime();

        // if the friction wheel launch speed is 0, use a default launch speed so ballistics
        // gives a reasonable computation
        float launchSpeed = frictionWheels.getPredictedLaunchSpeed();
        if (compareFloatClose(launchSpeed, 0.0f, 1e-5f))
        {
            launchSpeed = defaultLaunchSpeed;
        }

        auto &worldToTurret = transformer.getWorldToTurret(turretID);
        auto &worldToMajor = transformer.getWorldToTurretMajor();
        const Vector2f chassisVel = transformer.getChassisVelocity2d();

        // target state, frame whose axis is at the turret center and z is up
        // assume acceleration of the chassis is 0 since we don't measure it
        ballistics::SecondOrderKinematicState targetState = {
            modm::Vector3f{
                aimData.pva.xPos - worldToTurret.getX(),
                aimData.pva.yPos - worldToTurret.getY(),
                aimData.pva.zPos - worldToTurret.getZ()},
            modm::Vector3f{
                aimData.pva.xVel -
                    (chassisVel.x - turretMajor.getReadOnlyMotor().getChassisFrameVelocity() *
                                        std::cos(worldToMajor.getYaw()) * turretDistFromBase),
                aimData.pva.yVel -
                    (chassisVel.y - turretMajor.getReadOnlyMotor().getChassisFrameVelocity() *
                                        std::sin(worldToMajor.getYaw()) * turretDistFromBase),
                aimData.pva.zVel},
            modm::Vector3f{aimData.pva.xAcc, aimData.pva.yAcc, aimData.pva.zAcc},
        };

        // time in microseconds to project the target position ahead by
        int64_t projectForwardTimeDt =
            static_cast<int64_t>(solveStartTime) - static_cast<int64_t>(aimData.timestamp);
        const float latencyCompensationSeconds = projectForwardTimeDt / 1E6f;

        // project the target position forward in time s.t. we are computing a ballistics solution
        // for a target "now" rather than whenever the camera saw the target
        targetState.position = targetState.projectForward(latencyCompensationSeconds);

        lastComputedSolution = BallisticsSolution();
        lastComputedSolution->distance = targetState.position.getLength();

        const bool vacuumSolutionFound = ballistics::findTargetProjectileIntersection(
            targetState,
            launchSpeed,
            NUM_FORWARD_KINEMATIC_PROJECTIONS,
            &lastComputedSolution->pitchAngle,
            &lastComputedSolution->yawAngle,
            &lastComputedSolution->timeOfFlight,
            turretPitchOffset);

        if (!vacuumSolutionFound)
        {
            lastComputedSolution = std::nullopt;
            return std::nullopt;
        }

        float vacuumPitchAngle = lastComputedSolution->pitchAngle;
        float vacuumYawAngle = lastComputedSolution->yawAngle;
        float vacuumTimeOfFlight = lastComputedSolution->timeOfFlight;
        float vacuumDistance = lastComputedSolution->distance;

        float dragPitchAngle = vacuumPitchAngle;
        float dragYawAngle = vacuumYawAngle;
        float dragTimeOfFlight = vacuumTimeOfFlight;
        float dragDistance = vacuumDistance;
        bool dragSolutionFound = false;

        if (useDragCorrection)
        {
            dragSolutionFound = aruwsrc::algorithms::findTargetProjectileIntersectionWithSphereDrag(
                targetState,
                launchSpeed,
                DRAG_FORWARD_KINEMATIC_PROJECTIONS,
                &dragPitchAngle,
                &dragYawAngle,
                &dragTimeOfFlight,
                turretPitchOffset,
                &dragDistance);
        }

        if (dragSolutionFound)
        {
            lastComputedSolution->pitchAngle = dragPitchAngle;
            lastComputedSolution->yawAngle = dragYawAngle;
            lastComputedSolution->timeOfFlight = dragTimeOfFlight;
            lastComputedSolution->distance = dragDistance;
        }
    }

    return lastComputedSolution;
}
}  // namespace aruwsrc::sentry::algorithms
