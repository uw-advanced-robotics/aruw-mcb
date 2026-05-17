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

void logDragTelemetry(
    aruwsrc::communication::rtt::RttTelemetry *telemetry,
    uint8_t turretID,
    float launchSpeed,
    uint32_t dtMicroseconds,
    uint32_t totalSolveMicroseconds,
    uint32_t vacuumSolveMicroseconds,
    uint32_t dragSolveMicroseconds,
    bool vacuumSolutionFound,
    const SentryBallisticsSolver::DragComparison &comparison)
{
    if (telemetry == nullptr)
    {
        return;
    }

    telemetry->logSignal("bd:id", turretID);
    telemetry->logSignal("bd:dt", dtMicroseconds);
    telemetry->logSignal("bd:ust", totalSolveMicroseconds);
    telemetry->logSignal("bd:lat", comparison.latencyCompensationSeconds);
    telemetry->logSignal("bd:v0", launchSpeed);
    telemetry->logSignal(
        "bd:x",
        comparison.targetPositionX,
        comparison.targetPositionY,
        comparison.targetPositionZ);
    telemetry->logSignal(
        "bd:v",
        comparison.targetVelocityX,
        comparison.targetVelocityY,
        comparison.targetVelocityZ);
    telemetry->logSignal(
        "bd:a",
        comparison.targetAccelerationX,
        comparison.targetAccelerationY,
        comparison.targetAccelerationZ);

    telemetry->logSignal("bd:ok0", vacuumSolutionFound);
    telemetry->logSignal("bd:us0", vacuumSolveMicroseconds);
    telemetry->logSignal("bd:pt0", comparison.vacuumPitchAngle);
    telemetry->logSignal("bd:yw0", comparison.vacuumYawAngle);
    telemetry->logSignal("bd:tof0", comparison.vacuumTimeOfFlight);
    telemetry->logSignal("bd:d0", comparison.vacuumDistance);

    telemetry->logSignal("bd:ok", comparison.dragSolutionFound);
    telemetry->logSignal("bd:us", dragSolveMicroseconds);
    telemetry->logSignal("bd:pt", comparison.dragPitchAngle);
    telemetry->logSignal("bd:yw", comparison.dragYawAngle);
    telemetry->logSignal("bd:tof", comparison.dragTimeOfFlight);
    telemetry->logSignal("bd:d", comparison.dragDistance);
    telemetry->logSignal("bd:dpt", comparison.dragPitchAngle - comparison.vacuumPitchAngle);
    telemetry->logSignal("bd:dyw", comparison.dragYawAngle - comparison.vacuumYawAngle);
    telemetry->logSignal("bd:dtof", comparison.dragTimeOfFlight - comparison.vacuumTimeOfFlight);
    telemetry->logSignal("bd:dd", comparison.dragDistance - comparison.vacuumDistance);
}
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
    aruwsrc::communication::rtt::RttTelemetry *telemetry)
    : visionCoprocessor(visionCoprocessor),
      transformer(transformer),
      frictionWheels(frictionWheels),
      turretMajor(turretMajor),
      defaultLaunchSpeed(defaultLaunchSpeed),
      turretPitchOffset(turretPitchOffset),
      turretDistFromBase(turretDistFromBase),
      telemetry(telemetry),
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
        lastDragComparison = {};
        return std::nullopt;
    }

    if (lastAimDataTimestamp != aimData.timestamp ||
        lastOdometryTimestamp != transformer.getLastComputedOdometryTime())
    {
        const uint32_t solveStartTime = tap::arch::clock::getTimeMicroseconds();
        const uint32_t solveDt = lastSolveTimestamp == 0 ? 0 : solveStartTime - lastSolveTimestamp;
        lastSolveTimestamp = solveStartTime;

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
        lastDragComparison = {};
        lastDragComparison.launchSpeed = launchSpeed;
        lastDragComparison.latencyCompensationSeconds = latencyCompensationSeconds;
        lastDragComparison.targetPositionX = targetState.position.x;
        lastDragComparison.targetPositionY = targetState.position.y;
        lastDragComparison.targetPositionZ = targetState.position.z;
        lastDragComparison.targetVelocityX = targetState.velocity.x;
        lastDragComparison.targetVelocityY = targetState.velocity.y;
        lastDragComparison.targetVelocityZ = targetState.velocity.z;
        lastDragComparison.targetAccelerationX = targetState.acceleration.x;
        lastDragComparison.targetAccelerationY = targetState.acceleration.y;
        lastDragComparison.targetAccelerationZ = targetState.acceleration.z;

        const uint32_t vacuumSolveStartTime = tap::arch::clock::getTimeMicroseconds();
        const bool vacuumSolutionFound = ballistics::findTargetProjectileIntersection(
            targetState,
            launchSpeed,
            NUM_FORWARD_KINEMATIC_PROJECTIONS,
            &lastComputedSolution->pitchAngle,
            &lastComputedSolution->yawAngle,
            &lastComputedSolution->timeOfFlight,
            turretPitchOffset);
        const uint32_t vacuumSolveMicroseconds =
            tap::arch::clock::getTimeMicroseconds() - vacuumSolveStartTime;

        if (!vacuumSolutionFound)
        {
            logDragTelemetry(
                telemetry,
                turretID,
                launchSpeed,
                solveDt,
                tap::arch::clock::getTimeMicroseconds() - solveStartTime,
                vacuumSolveMicroseconds,
                0,
                false,
                lastDragComparison);
            lastComputedSolution = std::nullopt;
            return std::nullopt;
        }

        lastDragComparison.vacuumPitchAngle = lastComputedSolution->pitchAngle;
        lastDragComparison.vacuumYawAngle = lastComputedSolution->yawAngle;
        lastDragComparison.vacuumTimeOfFlight = lastComputedSolution->timeOfFlight;
        lastDragComparison.vacuumDistance = lastComputedSolution->distance;

        const uint32_t dragSolveStartTime = tap::arch::clock::getTimeMicroseconds();
        lastDragComparison.dragSolutionFound =
            aruwsrc::algorithms::findTargetProjectileIntersectionWithSphereDrag(
                targetState,
                launchSpeed,
                DRAG_FORWARD_KINEMATIC_PROJECTIONS,
                &lastDragComparison.dragPitchAngle,
                &lastDragComparison.dragYawAngle,
                &lastDragComparison.dragTimeOfFlight,
                turretPitchOffset,
                &lastDragComparison.dragDistance);
        const uint32_t dragSolveMicroseconds =
            tap::arch::clock::getTimeMicroseconds() - dragSolveStartTime;

        if (!lastDragComparison.dragSolutionFound)
        {
            lastDragComparison.dragPitchAngle = lastDragComparison.vacuumPitchAngle;
            lastDragComparison.dragYawAngle = lastDragComparison.vacuumYawAngle;
            lastDragComparison.dragTimeOfFlight = lastDragComparison.vacuumTimeOfFlight;
            lastDragComparison.dragDistance = lastDragComparison.vacuumDistance;
        }

        logDragTelemetry(
            telemetry,
            turretID,
            launchSpeed,
            solveDt,
            tap::arch::clock::getTimeMicroseconds() - solveStartTime,
            vacuumSolveMicroseconds,
            dragSolveMicroseconds,
            true,
            lastDragComparison);

        if (lastDragComparison.dragSolutionFound)
        {
            lastComputedSolution->pitchAngle = lastDragComparison.dragPitchAngle;
            lastComputedSolution->yawAngle = lastDragComparison.dragYawAngle;
            lastComputedSolution->timeOfFlight = lastDragComparison.dragTimeOfFlight;
            lastComputedSolution->distance = lastDragComparison.dragDistance;
        }
    }

    return lastComputedSolution;
}
}  // namespace aruwsrc::sentry::algorithms
