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

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/launcher/launch_speed_predictor_interface.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
using namespace tap::algorithms;
using namespace modm;

namespace aruwsrc::sentry::algorithms
{
SentryBallisticsSolver::SentryBallisticsSolver(
    const aruwsrc::communication::serial::VisionCoprocessor &visionCoprocessor,
    const odometry::SentryTransforms &transformer,
    const control::launcher::LaunchSpeedPredictorInterface &frictionWheels,
    const aruwsrc::control::turret::YawTurretSubsystem &turretMajor,
    const float defaultLaunchSpeed,
    const float turretPitchOffset,
    const float turretDistFromBase,
    const uint8_t turretID)
    : visionCoprocessor(visionCoprocessor),
      transformer(transformer),
      frictionWheels(frictionWheels),
      turretMajor(turretMajor),
      defaultLaunchSpeed(defaultLaunchSpeed),
      turretPitchOffset(turretPitchOffset),
      turretDistFromBase(turretDistFromBase),
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

        // World-frame position of this turret's pivot point
        const modm::Vector3f turretPosition(
            worldToTurret.getX(),
            worldToTurret.getY(),
            worldToTurret.getZ());

        // time in microseconds to project the target position ahead by
        int64_t projectForwardTimeDt =
            static_cast<int64_t>(tap::arch::clock::getTimeMicroseconds()) -
            static_cast<int64_t>(aimData.timestamp);

        // project the target position forward in time s.t. we are computing a ballistics
        // solution for a target "now" rather than whenever the camera saw the target
        auto projectedAimPosData = aimData.pva.projectForward(projectForwardTimeDt / 1E6f);

        // --- Strategy selection ---
        // Below OMEGA_THRESHOLD: jitter aim — evaluate all four plates, keep the one with
        // the shortest time-of-flight.
        // At or above OMEGA_THRESHOLD: pulse estimation — compute a shot-timing window for
        // whichever plate will be facing the turret at impact time.
        if (fabsf(projectedAimPosData.omega) < OMEGA_THRESHOLD)
        {
            // Effective chassis velocity at the turret minor position, accounting for the
            // turret major's rotation sweeping the minor around its axis.
            const modm::Vector2f effectiveChassisVel(
                chassisVel.x - turretMajor.getReadOnlyMotor().getChassisFrameVelocity() *
                                    std::cos(worldToMajor.getYaw()) * turretDistFromBase,
                chassisVel.y - turretMajor.getReadOnlyMotor().getChassisFrameVelocity() *
                                    std::sin(worldToMajor.getYaw()) * turretDistFromBase);

            lastComputedSolution = std::nullopt;

            for (int i = 0; i < 4; i++)
            {
                float currRadius =
                    (i % 2 == 0) ? projectedAimPosData.radius0 : projectedAimPosData.radius1;
                float currTheta = projectedAimPosData.theta + M_PI_2 * i;

                aruwsrc::communication::serial::VisionCoprocessor::RobotOrbitKinematicState
                    targetState(
                        {projectedAimPosData.xPos + currRadius * cos(currTheta) -
                             turretPosition.x,
                         projectedAimPosData.yPos + currRadius * sin(currTheta) -
                             turretPosition.y,
                         projectedAimPosData.zPos + projectedAimPosData.plateHeights[i] -
                             turretPosition.z},
                        {projectedAimPosData.xVel - effectiveChassisVel.x,
                         projectedAimPosData.yVel - effectiveChassisVel.y,
                         projectedAimPosData.zVel},
                        {projectedAimPosData.xAcc,
                         projectedAimPosData.yAcc,
                         projectedAimPosData.zAcc},
                        currRadius,
                        currTheta,
                        projectedAimPosData.omega);

                BallisticsSolution currentSolution = BallisticsSolution();
                currentSolution.distance = targetState.position.getLength();
                currentSolution.usePulseEstimation = false;
                currentSolution.activePlateIndex = i;
                currentSolution.shotWindowStart = 0;
                currentSolution.shotWindowEnd = 0;

                if (ballistics::findTargetProjectileIntersection(
                        targetState,
                        launchSpeed,
                        NUM_FORWARD_KINEMATIC_PROJECTIONS,
                        &currentSolution.pitchAngle,
                        &currentSolution.yawAngle,
                        &currentSolution.timeOfFlight,
                        turretPitchOffset) &&
                    (!lastComputedSolution ||
                     currentSolution.timeOfFlight < lastComputedSolution->timeOfFlight))
                {
                    lastComputedSolution = currentSolution;
                }
            }
        }
        else
        {
            // Fast-spinning target: use pulse estimation.

            // If we already have a valid pulse solution whose fire window is still open and
            // omega hasn't dropped, keep it to avoid thrashing the target plate selection.
            uint64_t currentTimeMicros = tap::arch::clock::getTimeMicroseconds();
            bool hasValidPulseSolution = lastComputedSolution.has_value() &&
                                         lastComputedSolution->usePulseEstimation &&
                                         currentTimeMicros <= lastComputedSolution->shotWindowEnd;

            if (hasValidPulseSolution)
            {
                // Fire window is still open; return the cached solution unchanged.
                return lastComputedSolution;
            }

            // Effective chassis velocity (same correction as above)
            const modm::Vector2f effectiveChassisVel(
                chassisVel.x - turretMajor.getReadOnlyMotor().getChassisFrameVelocity() *
                                    std::cos(worldToMajor.getYaw()) * turretDistFromBase,
                chassisVel.y - turretMajor.getReadOnlyMotor().getChassisFrameVelocity() *
                                    std::sin(worldToMajor.getYaw()) * turretDistFromBase);

            lastComputedSolution = computePulseEstimation(
                projectedAimPosData,
                turretPosition,
                effectiveChassisVel,
                launchSpeed);
        }
    }

    return lastComputedSolution;
}

std::optional<SentryBallisticsSolver::BallisticsSolution>
SentryBallisticsSolver::computePulseEstimation(
    const communication::serial::VisionCoprocessor::PositionData &projectedAimPosData,
    const modm::Vector3f &turretPosition,
    const modm::Vector2f &chassisVel,
    float launchSpeed)
{
    // Pulse Estimation:
    // 1. Estimate ToF using simple distance/launch_speed calculation
    // 2. Compute omega_total accounting for both rotation and translation
    // 3. Determine which plate will be "active" (shootable) at time ToF
    // 4. Compute accurate ballistics solution for active plate
    // 5. Calculate shot timing window accounting for plate width and omega_total

    float avgRadius = (projectedAimPosData.radius0 + projectedAimPosData.radius1) / 2.0f;
    modm::Vector3f robotCenterPos(
        projectedAimPosData.xPos + avgRadius * cos(projectedAimPosData.theta) - turretPosition.x,
        projectedAimPosData.yPos + avgRadius * sin(projectedAimPosData.theta) - turretPosition.y,
        projectedAimPosData.zPos - turretPosition.z);

    float approxDistance = robotCenterPos.getLength();
    float estimatedToF = approxDistance / launchSpeed;

    modm::Vector3f robotPos3D(
        projectedAimPosData.xPos - turretPosition.x,
        projectedAimPosData.yPos - turretPosition.y,
        projectedAimPosData.zPos - turretPosition.z);
    modm::Vector3f robotVel3D(
        projectedAimPosData.xVel - chassisVel.x,
        projectedAimPosData.yVel - chassisVel.y,
        projectedAimPosData.zVel);

    // Temporary state used only to call computeOmegaTotal / determineActivePlate helpers
    aruwsrc::communication::serial::VisionCoprocessor::RobotOrbitKinematicState tempState(
        robotPos3D,
        robotVel3D,
        {projectedAimPosData.xAcc, projectedAimPosData.yAcc, projectedAimPosData.zAcc},
        avgRadius,
        projectedAimPosData.theta,
        projectedAimPosData.omega);

    float omegaTotal = tempState.computeOmegaTotal(robotPos3D, robotVel3D);

    if (fabsf(omegaTotal) < 1e-6f)
    {
        return std::nullopt;
    }

    float aimAngle = atan2f(
        projectedAimPosData.yPos - turretPosition.y,
        projectedAimPosData.xPos - turretPosition.x);

    uint8_t activePlateIndex = tempState.determineActivePlate(
        omegaTotal,
        estimatedToF,
        PLATE_WIDTH,
        aimAngle,
        projectedAimPosData.theta);

    float activePlateHeight = projectedAimPosData.plateHeights[activePlateIndex];
    float activePlateRadius =
        (activePlateIndex % 2 == 0) ? projectedAimPosData.radius0 : projectedAimPosData.radius1;
    float activePlateTheta = projectedAimPosData.theta + activePlateIndex * M_PI_2;

    aruwsrc::communication::serial::VisionCoprocessor::RobotOrbitKinematicState activePlateState(
        {projectedAimPosData.xPos + activePlateRadius * cos(activePlateTheta) - turretPosition.x,
         projectedAimPosData.yPos + activePlateRadius * sin(activePlateTheta) - turretPosition.y,
         projectedAimPosData.zPos + activePlateHeight - turretPosition.z},
        {projectedAimPosData.xVel - chassisVel.x,
         projectedAimPosData.yVel - chassisVel.y,
         projectedAimPosData.zVel},
        {projectedAimPosData.xAcc, projectedAimPosData.yAcc, projectedAimPosData.zAcc},
        activePlateRadius,
        activePlateTheta,
        projectedAimPosData.omega);

    BallisticsSolution solution;
    solution.distance = activePlateState.position.getLength();
    solution.usePulseEstimation = true;
    solution.activePlateIndex = activePlateIndex;

    if (!ballistics::findTargetProjectileIntersection(
            activePlateState,
            launchSpeed,
            NUM_FORWARD_KINEMATIC_PROJECTIONS,
            &solution.pitchAngle,
            &solution.yawAngle,
            &solution.timeOfFlight,
            turretPitchOffset))
    {
        return std::nullopt;
    }

    // --- Shot timing window ---
    // Angular width subtended by one plate at the active radius
    float plateAngularWidth = PLATE_WIDTH / activePlateRadius;

    // Current angle of the active plate's centre
    float activePlateAngle = projectedAimPosData.theta + activePlateIndex * M_PI_2;

    // Angular offset from plate centre to aim line, normalised to [-π, π]
    float angularOffset = activePlateAngle - aimAngle;
    while (angularOffset > M_PI) angularOffset -= 2.0f * M_PI;
    while (angularOffset < -M_PI) angularOffset += 2.0f * M_PI;

    // Time for the plate centre to cross the aim line
    float timeToPlateCenterCrossing;
    if (omegaTotal > 0)
    {
        if (angularOffset <= 0) angularOffset += 2.0f * M_PI;
        timeToPlateCenterCrossing = angularOffset / omegaTotal;
    }
    else
    {
        if (angularOffset >= 0) angularOffset -= 2.0f * M_PI;
        timeToPlateCenterCrossing = angularOffset / omegaTotal;  // both negative → positive result
    }

    float halfWidthTime = (plateAngularWidth / 2.0f) / fabsf(omegaTotal);
    float timeToCloseEdge = timeToPlateCenterCrossing - halfWidthTime;
    float timeToFarEdge = timeToPlateCenterCrossing + halfWidthTime;

    // Convert to fire times: we need to launch early enough that the projectile arrives during the
    // crossing window.
    float fireWindowStart = timeToCloseEdge - solution.timeOfFlight;
    float fireWindowEnd = timeToFarEdge - solution.timeOfFlight;

    // Clamp to future only
    float startOffsetSeconds = (fireWindowStart > 0.0f) ? fireWindowStart : 0.0f;
    float endOffsetSeconds =
        (fireWindowEnd > startOffsetSeconds) ? fireWindowEnd : startOffsetSeconds;

    uint64_t currentTimeMicros = tap::arch::clock::getTimeMicroseconds();
    solution.shotWindowStart =
        currentTimeMicros + static_cast<uint64_t>(startOffsetSeconds * 1e6f);
    solution.shotWindowEnd = currentTimeMicros + static_cast<uint64_t>(endOffsetSeconds * 1e6f);

    return solution;
}
}  // namespace aruwsrc::sentry::algorithms
