/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "otto_ballistics_solver.hpp"

#include "tap/algorithms/ballistics.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/architecture/clock.hpp"

#include "aruwsrc/algorithms/spherical_projectile_aim.hpp"
#include "aruwsrc/communication/rtt/rtt_telemetry.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/launcher/launch_speed_predictor_interface.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/control/turret/robot_turret_subsystem.hpp"

using namespace tap::algorithms;
using namespace modm;

namespace aruwsrc::algorithms
{
namespace
{
static constexpr uint8_t DRAG_FORWARD_KINEMATIC_PROJECTIONS = 1;
}  // namespace

OttoBallisticsSolver::OttoBallisticsSolver(
    const aruwsrc::communication::serial::VisionCoprocessor &visionCoprocessor,
    const tap::algorithms::odometry::Odometry2DInterface &odometryInterface,
    const control::turret::RobotTurretSubsystem &turretSubsystem,
    const control::launcher::LaunchSpeedPredictorInterface &frictionWheels,
    const float defaultLaunchSpeed,
    const uint8_t turretID,
    aruwsrc::communication::rtt::RttTelemetry *,
    bool useDragCorrection)
    : visionCoprocessor(visionCoprocessor),
      odometryInterface(odometryInterface),
      turretSubsystem(turretSubsystem),
      frictionWheels(frictionWheels),
      defaultLaunchSpeed(defaultLaunchSpeed),
      useDragCorrection(useDragCorrection),
      turretID(turretID)
{
}

std::optional<OttoBallisticsSolver::BallisticsSolution> OttoBallisticsSolver::
    computeTurretAimAngles()
{
    const auto &aimData = visionCoprocessor.getLastAimData(turretID);
    debug.cvOnline = visionCoprocessor.isCvOnline();
    debug.aimDataUpdated = aimData.pva.updated;
    debug.useDragCorrection = useDragCorrection;
    debug.aimDataTimestamp = aimData.timestamp;
    debug.odometryTimestamp = odometryInterface.getLastComputedOdometryTime();

    // Verify that CV is actually online and that the aimData had a target
    if (!debug.cvOnline || !debug.aimDataUpdated)
    {
        debug.vacuumSolutionFound = false;
        debug.dragSolutionFound = false;
        lastComputedSolution = std::nullopt;
        return std::nullopt;
    }

    if (lastAimDataTimestamp != aimData.timestamp ||
        lastOdometryTimestamp != odometryInterface.getLastComputedOdometryTime())
    {
        const uint32_t solveStartTime = tap::arch::clock::getTimeMicroseconds();
        const uint32_t solveDt = lastSolveTimestamp == 0 ? 0 : solveStartTime - lastSolveTimestamp;
        lastSolveTimestamp = solveStartTime;
        debug.sequence++;
        debug.solveDtMicroseconds = solveDt;

        lastAimDataTimestamp = aimData.timestamp;
        lastOdometryTimestamp = odometryInterface.getLastComputedOdometryTime();

        // if the friction wheel launch speed is 0, use a default launch speed so ballistics
        // gives a reasonable computation
        float launchSpeed = frictionWheels.getPredictedLaunchSpeed();
        if (compareFloatClose(launchSpeed, 0.0f, 1e-5f))
        {
            launchSpeed = defaultLaunchSpeed;
        }
        debug.launchSpeed = launchSpeed;
        debug.reynoldsNumber = ROBO_MASTER_17MM_SPHERE.reynoldsNumber(launchSpeed);
        debug.dragCoefficient = ROBO_MASTER_17MM_SPHERE.dragCoefficient(launchSpeed);
        debug.dragAccelerationScale = ROBO_MASTER_17MM_SPHERE.dragAccelerationScale(launchSpeed);
        debug.dragRate = debug.dragAccelerationScale * launchSpeed;

        // defines the turret where the chassis is, under the assumption that the chassis origin and
        // turret origin coincide
        modm::Vector3f turretPosition =
            modm::Vector3f(odometryInterface.getCurrentLocation2D().getPosition(), 0);

        // Puts turret in it's place in world frame
        // If no offset, skip all offsetting
        if (turretSubsystem.getTurretOffset() != modm::Vector3f(0, 0, 0))
        {
            // make this in here to minimize resource usage I guess
            modm::Vector3f turretOffset = turretSubsystem.getTurretOffset();
            // yaw is 0, so chassis frame and world frame share orientation. They may not share
            // translation, so we still need to add that.
            if (compareFloatClose(odometryInterface.getYaw(), 0.0f, 1e-5f))
            {
                // Assume that z is parallel to yaw and needs not adjusting.
                // This breaks if the robot rolls, but we'd need to implement 3D odometry anyways
                // soooo not my problem! For now, skips 3D vector rotation.
                rotateVector(&turretOffset.x, &turretOffset.y, odometryInterface.getYaw());
            }
            turretPosition += turretOffset;
        }

        const Vector2f chassisVel = odometryInterface.getCurrentVelocity2D();
        const modm::Vector3f aimPosition(aimData.pva.xPos, aimData.pva.yPos, aimData.pva.zPos);
        const modm::Vector3f relativeTargetPosition = aimPosition - turretPosition;
        debug.aimPositionX = aimPosition.x;
        debug.aimPositionY = aimPosition.y;
        debug.aimPositionZ = aimPosition.z;
        debug.turretPositionX = turretPosition.x;
        debug.turretPositionY = turretPosition.y;
        debug.turretPositionZ = turretPosition.z;
        debug.relativeTargetX = relativeTargetPosition.x;
        debug.relativeTargetY = relativeTargetPosition.y;
        debug.relativeTargetZ = relativeTargetPosition.z;

        // target state, frame whose axis is at the turret center and z is up
        // assume acceleration of the chassis is 0 since we don't measure it

        ballistics::SecondOrderKinematicState targetState(
            relativeTargetPosition,
            modm::Vector3f(
                aimData.pva.xVel - chassisVel.x,
                aimData.pva.yVel - chassisVel.y,
                aimData.pva.zVel),
            modm::Vector3f(
                aimData.pva.xAcc,
                aimData.pva.yAcc,
                aimData.pva.zAcc)  // TODO consider using chassis
                                   // acceleration from IMU
        );

        // time in microseconds to project the target position ahead by
        int64_t projectForwardTimeDt =
            static_cast<int64_t>(solveStartTime) - static_cast<int64_t>(aimData.timestamp);
        const float latencyCompensationSeconds = projectForwardTimeDt / 1E6f;
        debug.latencyCompensationSeconds = latencyCompensationSeconds;

        // project the target position forward in time s.t. we are computing a ballistics solution
        // for a target "now" rather than whenever the camera saw the target
        targetState.position = targetState.projectForward(latencyCompensationSeconds);
        debug.projectedTargetX = targetState.position.x;
        debug.projectedTargetY = targetState.position.y;
        debug.projectedTargetZ = targetState.position.z;
        debug.targetVelocityX = targetState.velocity.x;
        debug.targetVelocityY = targetState.velocity.y;
        debug.targetVelocityZ = targetState.velocity.z;
        debug.horizontalDistance = hypotf(targetState.position.x, targetState.position.y) +
                                   turretSubsystem.getPitchOffset();

        lastComputedSolution = BallisticsSolution();
        lastComputedSolution->distance = targetState.position.getLength();

        const uint32_t vacuumSolveStartTime = tap::arch::clock::getTimeMicroseconds();
        const bool vacuumSolutionFound = ballistics::findTargetProjectileIntersection(
            targetState,
            launchSpeed,
            NUM_FORWARD_KINEMATIC_PROJECTIONS,
            &lastComputedSolution->pitchAngle,
            &lastComputedSolution->yawAngle,
            &lastComputedSolution->timeOfFlight,
            turretSubsystem.getPitchOffset());
        const uint32_t vacuumSolveMicroseconds =
            tap::arch::clock::getTimeMicroseconds() - vacuumSolveStartTime;
        debug.vacuumSolveMicroseconds = vacuumSolveMicroseconds;
        debug.vacuumSolutionFound = vacuumSolutionFound;
        debug.vacuumPitch = lastComputedSolution->pitchAngle;
        debug.vacuumYaw = lastComputedSolution->yawAngle;
        debug.vacuumTimeOfFlight = lastComputedSolution->timeOfFlight;
        debug.vacuumDistance = lastComputedSolution->distance;
        debug.dragSolutionFound = false;
        debug.dragSolveMicroseconds = 0;
        debug.dragPitch = lastComputedSolution->pitchAngle;
        debug.dragYaw = lastComputedSolution->yawAngle;
        debug.dragTimeOfFlight = lastComputedSolution->timeOfFlight;
        debug.dragDistance = lastComputedSolution->distance;
        debug.deltaPitch = 0.0f;
        debug.deltaYaw = 0.0f;
        debug.deltaTimeOfFlight = 0.0f;
        debug.deltaDistance = 0.0f;
        debug.dragHorizontalVelocity = launchSpeed * cosf(debug.vacuumPitch);
        debug.dragRemainingHorizontalVelocityRatio =
            debug.dragHorizontalVelocity > 0.0f
                ? 1.0f - debug.dragRate * debug.horizontalDistance / debug.dragHorizontalVelocity
                : 0.0f;
        auto estimatedDragTimeOfFlight = estimateHorizontalDragTimeOfFlight(
            debug.horizontalDistance,
            debug.dragHorizontalVelocity,
            debug.dragAccelerationScale,
            launchSpeed);
        debug.dragEstimatedTimeOfFlight =
            estimatedDragTimeOfFlight.has_value() ? *estimatedDragTimeOfFlight : 0.0f;

        if (!vacuumSolutionFound)
        {
            debug.totalSolveMicroseconds = tap::arch::clock::getTimeMicroseconds() - solveStartTime;
            lastComputedSolution = std::nullopt;
        }
        else if (!useDragCorrection)
        {
            debug.totalSolveMicroseconds = tap::arch::clock::getTimeMicroseconds() - solveStartTime;
        }
        else
        {
            const BallisticsSolution vacuumSolution = *lastComputedSolution;
            BallisticsSolution dragSolution = vacuumSolution;

            const uint32_t dragSolveStartTime = tap::arch::clock::getTimeMicroseconds();
            const bool dragSolutionFound = findTargetProjectileIntersectionWithSphereDrag(
                targetState,
                launchSpeed,
                DRAG_FORWARD_KINEMATIC_PROJECTIONS,
                &dragSolution.pitchAngle,
                &dragSolution.yawAngle,
                &dragSolution.timeOfFlight,
                turretSubsystem.getPitchOffset(),
                &dragSolution.distance);
            const uint32_t dragSolveMicroseconds =
                tap::arch::clock::getTimeMicroseconds() - dragSolveStartTime;
            debug.dragSolveMicroseconds = dragSolveMicroseconds;
            debug.dragSolutionFound = dragSolutionFound;
            debug.dragPitch = dragSolution.pitchAngle;
            debug.dragYaw = dragSolution.yawAngle;
            debug.dragTimeOfFlight = dragSolution.timeOfFlight;
            debug.dragDistance = dragSolution.distance;
            debug.deltaPitch = dragSolution.pitchAngle - vacuumSolution.pitchAngle;
            debug.deltaYaw = dragSolution.yawAngle - vacuumSolution.yawAngle;
            debug.deltaTimeOfFlight = dragSolution.timeOfFlight - vacuumSolution.timeOfFlight;
            debug.deltaDistance = dragSolution.distance - vacuumSolution.distance;
            debug.dragHorizontalVelocity = launchSpeed * cosf(dragSolution.pitchAngle);
            debug.dragRemainingHorizontalVelocityRatio =
                debug.dragHorizontalVelocity > 0.0f
                    ? 1.0f -
                          debug.dragRate * debug.horizontalDistance / debug.dragHorizontalVelocity
                    : 0.0f;
            estimatedDragTimeOfFlight = estimateHorizontalDragTimeOfFlight(
                debug.horizontalDistance,
                debug.dragHorizontalVelocity,
                debug.dragAccelerationScale,
                launchSpeed);
            debug.dragEstimatedTimeOfFlight =
                estimatedDragTimeOfFlight.has_value() ? *estimatedDragTimeOfFlight : 0.0f;
            debug.totalSolveMicroseconds = tap::arch::clock::getTimeMicroseconds() - solveStartTime;

            if (dragSolutionFound)
            {
                lastComputedSolution = dragSolution;
            }
        }
    }

    return lastComputedSolution;
}
}  // namespace aruwsrc::algorithms
