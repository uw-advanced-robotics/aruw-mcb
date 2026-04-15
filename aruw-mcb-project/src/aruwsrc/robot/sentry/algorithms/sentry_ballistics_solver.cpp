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

#include "aruwsrc/algorithms/spherical_projectile_aim.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/launcher/launch_speed_predictor_interface.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
using namespace tap::algorithms;
using namespace modm;

namespace aruwsrc::sentry::algorithms
{
static constexpr float PITCH_CORRECTION_LOW_PASS_ALPHA = 0.05f;

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
    debugTargetValid = false;
    debugBaseSolutionValid = false;
    debugDragSolutionAttempted = false;
    debugDragSolutionAccepted = false;
    debugLaunchSpeed = 0.0f;
    debugTargetPositionX = 0.0f;
    debugTargetPositionY = 0.0f;
    debugTargetPositionZ = 0.0f;
    debugBasePitchAngle = 0.0f;
    debugBaseYawAngle = 0.0f;
    debugBaseTimeOfFlight = 0.0f;
    debugBaseDistance = 0.0f;
    debugCorrectedPitchAngle = 0.0f;
    debugCorrectedYawAngle = 0.0f;
    debugCorrectedTimeOfFlight = 0.0f;
    debugCorrectedDistance = 0.0f;
    debugFinalPitchAngle = 0.0f;
    debugFinalYawAngle = 0.0f;
    debugFinalTimeOfFlight = 0.0f;
    debugFinalDistance = 0.0f;
    debugBaseVerticalError = 0.0f;
    debugCorrectedVerticalError = 0.0f;
    debugConstantPitchCorrection = 0.0f;
    debugAdditionalPitchCorrection = 0.0f;
    debugDesiredPitchCorrection = 0.0f;
    debugAppliedPitchCorrection = 0.0f;
    debugPitchCorrection = 0.0f;
    debugYawCorrection = 0.0f;
    debugTimeOfFlightCorrection = 0.0f;

    const auto &aimData = visionCoprocessor.getLastAimData(turretID);
    // Verify that CV is actually online and that the aimData had a target
    if (!visionCoprocessor.isCvOnline() || !aimData.pva.updated)
    {
        pitchCorrectionLatched = false;
        appliedPitchCorrection = 0.0f;
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
        debugLaunchSpeed = launchSpeed;

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
            static_cast<int64_t>(tap::arch::clock::getTimeMicroseconds()) -
            static_cast<int64_t>(aimData.timestamp);

        // project the target position forward in time s.t. we are computing a ballistics solution
        // for a target "now" rather than whenever the camera saw the target
        targetState.position = targetState.projectForward(projectForwardTimeDt / 1E6f);
        debugTargetValid = true;
        debugTargetPositionX = targetState.position.x;
        debugTargetPositionY = targetState.position.y;
        debugTargetPositionZ = targetState.position.z;

        BallisticsSolution baseSolution{};
        baseSolution.distance = targetState.position.getLength();

        if (!ballistics::findTargetProjectileIntersection(
                targetState,
                launchSpeed,
                NUM_FORWARD_KINEMATIC_PROJECTIONS,
                &baseSolution.pitchAngle,
                &baseSolution.yawAngle,
                &baseSolution.timeOfFlight,
                turretPitchOffset))
        {
            pitchCorrectionLatched = false;
            appliedPitchCorrection = 0.0f;
            lastComputedSolution = std::nullopt;
        }
        else
        {
            debugBaseSolutionValid = true;
            debugBasePitchAngle = baseSolution.pitchAngle;
            debugBaseYawAngle = baseSolution.yawAngle;
            debugBaseTimeOfFlight = baseSolution.timeOfFlight;
            debugBaseDistance = baseSolution.distance;
            debugCorrectedPitchAngle = debugBasePitchAngle;
            debugCorrectedYawAngle = debugBaseYawAngle;
            debugCorrectedTimeOfFlight = debugBaseTimeOfFlight;
            debugCorrectedDistance = debugBaseDistance;
            debugFinalPitchAngle = debugBasePitchAngle;
            debugFinalYawAngle = debugBaseYawAngle;
            debugFinalTimeOfFlight = debugBaseTimeOfFlight;
            debugFinalDistance = debugBaseDistance;

            BallisticsSolution finalSolution = baseSolution;
            BallisticsSolution dragCorrectedSolution = baseSolution;
            debugDragSolutionAttempted = true;

            const modm::Vector3f compensatedTargetPosition =
                targetState.projectForward(baseSolution.timeOfFlight);
            const float horizontalDistance = sqrtf(
                compensatedTargetPosition.x * compensatedTargetPosition.x +
                compensatedTargetPosition.y * compensatedTargetPosition.y);

            const auto baseIntersection = aruwsrc::algorithms::simulateSphereDragIntersection(
                horizontalDistance,
                compensatedTargetPosition.z,
                launchSpeed,
                baseSolution.pitchAngle,
                turretPitchOffset);
            if (baseIntersection.has_value())
            {
                debugBaseVerticalError = baseIntersection->verticalError;
            }

            if (aruwsrc::algorithms::applySphereDragBallisticsCompensation(
                    targetState,
                    launchSpeed,
                    &dragCorrectedSolution.pitchAngle,
                    &dragCorrectedSolution.yawAngle,
                    &dragCorrectedSolution.timeOfFlight,
                    &dragCorrectedSolution.distance,
                    turretPitchOffset))
            {
                const float desiredPitchCorrection =
                    dragCorrectedSolution.pitchAngle - debugBasePitchAngle;

                if (!pitchCorrectionLatched)
                {
                    appliedPitchCorrection = desiredPitchCorrection;
                    pitchCorrectionLatched = true;
                }
                else
                {
                    appliedPitchCorrection +=
                        PITCH_CORRECTION_LOW_PASS_ALPHA *
                        (desiredPitchCorrection - appliedPitchCorrection);
                }

                dragCorrectedSolution.pitchAngle = debugBasePitchAngle + appliedPitchCorrection;
                debugCorrectedPitchAngle = dragCorrectedSolution.pitchAngle;
                debugCorrectedYawAngle = debugBaseYawAngle;
                debugCorrectedTimeOfFlight = debugBaseTimeOfFlight;
                debugCorrectedDistance = dragCorrectedSolution.distance;
                debugConstantPitchCorrection =
                    aruwsrc::algorithms::DEFAULT_SPHERE_DRAG_CORRECTION_CONFIG
                        .constantPitchOffsetRadians;
                debugDesiredPitchCorrection = desiredPitchCorrection;
                debugAppliedPitchCorrection = appliedPitchCorrection;
                debugPitchCorrection = appliedPitchCorrection;
                debugAdditionalPitchCorrection =
                    desiredPitchCorrection - debugConstantPitchCorrection;
                debugYawCorrection = 0.0f;
                debugTimeOfFlightCorrection = 0.0f;

                const auto correctedIntersection =
                    aruwsrc::algorithms::simulateSphereDragIntersection(
                        horizontalDistance,
                        compensatedTargetPosition.z,
                        launchSpeed,
                        dragCorrectedSolution.pitchAngle,
                        turretPitchOffset);
                if (correctedIntersection.has_value())
                {
                    debugCorrectedVerticalError = correctedIntersection->verticalError;
                }

                finalSolution = dragCorrectedSolution;
                debugDragSolutionAccepted = true;
                debugFinalPitchAngle = dragCorrectedSolution.pitchAngle;
                debugFinalYawAngle = debugBaseYawAngle;
                debugFinalTimeOfFlight = debugBaseTimeOfFlight;
                debugFinalDistance = dragCorrectedSolution.distance;
            }

            lastComputedSolution = finalSolution;
        }
    }
    else
    {
        debugAppliedPitchCorrection = appliedPitchCorrection;
        debugPitchCorrection = appliedPitchCorrection;
    }

    return lastComputedSolution;
}
}  // namespace aruwsrc::sentry::algorithms
