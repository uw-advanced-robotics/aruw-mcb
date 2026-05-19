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

#include "cv_ballistics_solver.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/algorithms/wrapped_float.hpp"

#include "aruwsrc/algorithms/robot_target_kinematic_state.hpp"
#include "aruwsrc/communication/rtt/rtt_telemetry.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/launcher/launch_speed_predictor_interface.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/control/turret/robot_turret_subsystem.hpp"

using namespace tap::algorithms;

using tap::algorithms::Angle;
using tap::algorithms::WrappedFloat;
using tap::algorithms::ballistics::SecondOrderKinematicState;

namespace aruwsrc::algorithms
{
CvBallisticsSolver::CvBallisticsSolver(
    const aruwsrc::communication::serial::VisionCoprocessor& visionCoprocessor,
    const aruwsrc::algorithms::odometry::transforms::TransformerInterface& transformer,
    const control::launcher::LaunchSpeedPredictorInterface& frictionWheels,
    const float defaultLaunchSpeed,
    const uint8_t turretID,
    float turretPitchOffset,
    aruwsrc::communication::rtt::RttTelemetry* telemetry)
    : visionCoprocessor(visionCoprocessor),
      transformer(transformer),
      worldToTurret(transformer.getWorldToTurret(turretID)),
      frictionWheels(frictionWheels),
      defaultLaunchSpeed(defaultLaunchSpeed),
      turretPitchOffset(turretPitchOffset),
      turretID(turretID),
      telemetry(telemetry)
{
}

std::optional<CvBallisticsSolver::BallisticsSolution> CvBallisticsSolver::computeTurretAimAngles()
{
    const auto& aimData = visionCoprocessor.getLastAimData(turretID);

    if (telemetry)
    {
        telemetry->logSignal("ballistics:cv_online", visionCoprocessor.isCvOnline());
        telemetry->logSignal("ballistics:aim_updated", aimData.pva.updated);
    }

    // Verify that CV is actually online and that the aimData had a target
    if (!visionCoprocessor.isCvOnline() || !aimData.pva.updated)
    {
        lastComputedSolution = std::nullopt;
        return std::nullopt;
    }

    if (lastAimDataTimestamp == aimData.timestamp &&
        lastOdometryTimestamp == transformer.getLastComputedOdometryTime())
    {
        return lastComputedSolution;
    }

    lastAimDataTimestamp = aimData.timestamp;
    lastOdometryTimestamp = transformer.getLastComputedOdometryTime();

    // if the friction wheel launch speed is 0, use a default launch speed so ballistics
    // gives a reasonable computation
    float launchSpeed = frictionWheels.getPredictedLaunchSpeed();
    if (compareFloatClose(launchSpeed, 0.0f, 1e-5f))
    {
        launchSpeed = defaultLaunchSpeed;
    }

    if (telemetry)
    {
        telemetry->logSignal("ballistics:launch_speed", launchSpeed);
    }

    // time in microseconds to project the target position ahead by
    int64_t projectForwardTimeDt = static_cast<int64_t>(tap::arch::clock::getTimeMicroseconds()) -
                                   static_cast<int64_t>(aimData.timestamp);

    // project the target position forward in time s.t. we are computing a ballistics
    // solution for a target "now" rather than whenever the camera saw the target
    aruwsrc::communication::serial::VisionCoprocessor::PositionData projectedAimPosData =
        aimData.pva.projectForward(projectForwardTimeDt / 1E6f);

    if (telemetry)
    {
        telemetry->logSignal(
            "ballistics:target_pos",
            projectedAimPosData.xPos,
            projectedAimPosData.yPos,
            projectedAimPosData.zPos);
        telemetry->logSignal(
            "ballistics:target_vel",
            projectedAimPosData.xVel,
            projectedAimPosData.yVel,
            projectedAimPosData.zVel);
        telemetry->logSignal("ballistics:omega", projectedAimPosData.omega);
        telemetry->logSignal("ballistics:theta", projectedAimPosData.theta);
    }

    omegaLP = omegaLPAlpha * projectedAimPosData.omega + (1 - omegaLPAlpha) * omegaLP;

    // Use enemy angular velocity to determine which aiming strategy to use
    // TODO: this should technically be the angular velocity in the rotating target-tracking
    // frame ("omegaTotal")
    if (fabsf(omegaLP) < OMEGA_THRESHOLD)
    {
        // Jitter Aim
        lastComputedSolution = std::nullopt;
        for (int i = 0; i < 4; i++)
        {
            float currRadius =
                (i % 2 == 0) ? projectedAimPosData.radius0 : projectedAimPosData.radius1;
            float currTheta = projectedAimPosData.theta + M_PI_2 * i;

            RobotTargetKinematicState targetState(
                {projectedAimPosData.xPos + currRadius * cos(currTheta) - worldToTurret.getX(),
                 projectedAimPosData.yPos + currRadius * sin(currTheta) - worldToTurret.getY(),
                 projectedAimPosData.zPos + projectedAimPosData.plateHeights[i] -
                     worldToTurret.getZ()},
                {projectedAimPosData.xVel - worldToTurret.getXVel(),
                 projectedAimPosData.yVel - worldToTurret.getYVel(),
                 projectedAimPosData.zVel},
                {projectedAimPosData.xAcc, projectedAimPosData.yAcc, projectedAimPosData.zAcc},
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
                if (telemetry)
                {
                    telemetry->logSignal("ballistics:jitter_best_plate", static_cast<int>(i));
                    telemetry->logSignal("ballistics:jitter_tof", currentSolution.timeOfFlight);
                }
            }
        }

        if (telemetry)
        {
            if (lastComputedSolution)
            {
                telemetry->logSignal("ballistics:solution_found", true);
                telemetry->logSignal("ballistics:yaw", lastComputedSolution->yawAngle);
                telemetry->logSignal("ballistics:pitch", lastComputedSolution->pitchAngle);
                telemetry->logSignal("ballistics:distance", lastComputedSolution->distance);
            }
        }
    }
    else
    {
        // Shot Timing
        // Use pulse estimation for fast rotating targets

        // Check if we already have a valid pulse estimation solution with an open fire window
        uint64_t currentTimeMicros = tap::arch::clock::getTimeMicroseconds();
        bool hasValidPulseSolution = lastComputedSolution.has_value() &&
                                     lastComputedSolution->usePulseEstimation &&
                                     currentTimeMicros <= lastComputedSolution->shotWindowEnd;

        // Discard pulse solution if omega has dropped below threshold
        //   (shouldn't ever happen bc we don't consider angular acceleration when projecting
        //   forward)
        bool omegaBelowThreshold = fabsf(projectedAimPosData.omega) < OMEGA_THRESHOLD;

        if (hasValidPulseSolution && !omegaBelowThreshold && false)
        {
            // Recompute the aim solution so pitch/yaw can track vertical motion, but keep the
            // existing pulse timing window so shot timing remains stable.
            float activePlateHeight =
                projectedAimPosData.plateHeights[lastComputedSolution->activePlateIndex];

            SecondOrderKinematicState robotCenterState(
                {projectedAimPosData.xPos - worldToTurret.getX(),
                 projectedAimPosData.yPos - worldToTurret.getY(),
                 projectedAimPosData.zPos + activePlateHeight - worldToTurret.getZ()},
                {projectedAimPosData.xVel - worldToTurret.getXVel(),
                 projectedAimPosData.yVel - worldToTurret.getYVel(),
                 projectedAimPosData.zVel},
                {projectedAimPosData.xAcc, projectedAimPosData.yAcc, projectedAimPosData.zAcc});

            BallisticsSolution updatedSolution = *lastComputedSolution;
            updatedSolution.distance = robotCenterState.position.getLength();

            if (ballistics::findTargetProjectileIntersection(
                    robotCenterState,
                    launchSpeed,
                    NUM_FORWARD_KINEMATIC_PROJECTIONS,
                    &updatedSolution.pitchAngle,
                    &updatedSolution.yawAngle,
                    &updatedSolution.timeOfFlight,
                    turretPitchOffset))
            {
                lastComputedSolution = updatedSolution;

                if (telemetry)
                {
                    uint64_t timeRemaining =
                        lastComputedSolution->shotWindowEnd - currentTimeMicros;
                    telemetry->logSignal(
                        "ballistics:pulse_window_remaining_us",
                        static_cast<float>(timeRemaining));
                }

                return lastComputedSolution;
            }
        }
        else
        {
            // Either no existing solution, fire window closed, or omega dropped, so compute new
            // solution
            lastComputedSolution = computePulseEstimation(projectedAimPosData, launchSpeed);
        }
    }

    return lastComputedSolution;
}

std::optional<CvBallisticsSolver::BallisticsSolution> CvBallisticsSolver::computePulseEstimation(
    const communication::serial::VisionCoprocessor::PositionData& projectedAimPosData,
    float launchSpeed)
{
    // Pulse Estimation:
    // 1. Estimate ToF using simple distance/launch_speed calculation
    // 2. Compute omega_total accounting for both rotation and translation
    // 3. Determine which plate will be "active" (shootable) at time ToF
    // 4. Compute accurate ballistics solution for active plate
    // 5. Calculate shot timing window accounting for plate width and omega_total

    // Estimate approximate distance and ToF to nearest point on robot perimeter
    float avgRadius = (projectedAimPosData.radius0 + projectedAimPosData.radius1) / 2.0f;

    modm::Vector3f robotPos(
        projectedAimPosData.xPos - worldToTurret.getX(),
        projectedAimPosData.yPos - worldToTurret.getY(),
        projectedAimPosData.zPos - worldToTurret.getZ());

    modm::Vector3f robotVel(
        projectedAimPosData.xVel - worldToTurret.getXVel(),
        projectedAimPosData.yVel - worldToTurret.getYVel(),
        projectedAimPosData.zVel);

    float horizontalDistToClosestPoint = robotPos.xy().getLength() - avgRadius;
    float approxDistance = modm::Vector2f(horizontalDistToClosestPoint, robotPos.z).getLength();
    float estimatedToF = approxDistance / launchSpeed;
    // TODO: could do a center ballistics pass instead? would account for turret pitch

    auto estHitTimePosData = projectedAimPosData.projectForward(estimatedToF);

    // Compute the angular velocity of the target wrt a rotating frame who's x axis always faces
    // the target (e.g. the turret tracks the target robot center)
    //
    // $$ omega_{total} = omega_{robot} + \frac{(r \times v)_z}{|r|^2} $$

    // Compute cross product (r x v)_z component
    float crossProductZ = robotPos.x * robotVel.y - robotPos.y * robotVel.x;

    // Magnitude squared of r (in x-y plane)
    float rMagSquared = robotPos.xy().getLengthSquared();

    float omegaFromTranslation = rMagSquared < 1e-6f ? 0 : crossProductZ / rMagSquared;

    // TODO: is translation-induced component even necessary
    float omegaTotal = projectedAimPosData.omega + omegaFromTranslation;

    if (telemetry)
    {
        telemetry->logSignal("ballistics:pulse_omega_robot", projectedAimPosData.omega);
        telemetry->logSignal("ballistics:pulse_omega_total", omegaTotal);
    }

    // Avoid division by zero
    if (fabsf(omegaTotal) < 1e-6f)
    {
        if (telemetry)
        {
            telemetry->println("[ballistics] Pulse estimation failed: omega_total near zero");
        }
        return std::nullopt;
    }

    // Calculate our aim angle (from turret to predicted robot center)
    float aimAngle = atan2f(
        estHitTimePosData.yPos - worldToTurret.getY(),
        estHitTimePosData.xPos - worldToTurret.getX());

    // Determine active plate based on omega_total and estimated ToF
    // At the time we expect the projectile to hit the robot, we want to choose the plate in a
    // quadrant facing us.
    // Ideally, the precise bounds of the quadrant shouldn't matter (it only affects specifically
    // when we decide to target the next plate in between shot windows), but it could be biased
    // towards the direction the plate arrives all the way until the closing edge is a plate's width
    // away from the aim line.
    // float avgPlateAngularWidth = PLATE_WIDTH / avgRadius;
    float desiredPlateQuadrantStart = -M_PI_4;
    // float desiredPlateQuadrantStart =
    //     omegaTotal > 0 ? avgPlateAngularWidth / 2 - M_PI_2 : -avgPlateAngularWidth / 2;
    WrappedFloat aimLineToProjectedPlate0 =
        WrappedFloat(estHitTimePosData.theta, 0, M_TWOPI) - aimAngle + M_PI;
    float desiredPlateQuadrantStartToProjectedPlate0 =
        (Angle(-desiredPlateQuadrantStart) - aimLineToProjectedPlate0).getWrappedValue();
    uint8_t activePlateIndex =
        static_cast<uint8_t>(desiredPlateQuadrantStartToProjectedPlate0 / M_PI_2);

    // float quadrantStartToActivePlate = fmodf(desiredPlateQuadrantStartToProjectedPlate0, M_PI_2);
    // float projectedActivePlateToAimLine = -desiredPlateQuadrantStart -
    // quadrantStartToActivePlate;

    if (telemetry)
    {
        telemetry->logSignal("ballistics:pulse_active_plate", static_cast<int>(activePlateIndex));
        telemetry->logSignal("ballistics:pulse_estimated_tof", estimatedToF);
        telemetry->logSignal("ballistics:pulse_aim_angle", aimAngle);
    }

    // Aim at active plate's current position
    // The RobotTargetKinematicState model will handle projecting both:
    // 1. Linear motion of robot center (constant acceleration)
    // 2. Rotational motion of the plate around the center (constant angular velocity)
    float activePlateHeight = projectedAimPosData.plateHeights[activePlateIndex];
    float activePlateRadius =
        (activePlateIndex % 2 == 0) ? projectedAimPosData.radius0 : projectedAimPosData.radius1;

    // Robot center's current position, velocity, acceleration
    SecondOrderKinematicState robotCenterState(
        {projectedAimPosData.xPos - worldToTurret.getX(),
         projectedAimPosData.yPos - worldToTurret.getY(),
         projectedAimPosData.zPos + activePlateHeight - worldToTurret.getZ()},
        {projectedAimPosData.xVel - worldToTurret.getXVel(),
         projectedAimPosData.yVel - worldToTurret.getYVel(),
         projectedAimPosData.zVel},
        {projectedAimPosData.xAcc, projectedAimPosData.yAcc, projectedAimPosData.zAcc});

    BallisticsSolution solution;
    solution.distance = robotCenterState.position.getLength();
    solution.usePulseEstimation = true;
    solution.activePlateIndex = activePlateIndex;

    if (!ballistics::findTargetProjectileIntersection(
            robotCenterState,
            launchSpeed,
            NUM_FORWARD_KINEMATIC_PROJECTIONS,
            &solution.pitchAngle,
            &solution.yawAngle,
            &solution.timeOfFlight,
            turretPitchOffset -
                activePlateRadius))  // aim at nearest point on perimeter by pretending the turret
                                     //   pitch axis is offset forward by the target plate radius
    {
        if (telemetry)
        {
            telemetry->println("[ballistics] Pulse estimation: no projectile intersection found");
        }
        return std::nullopt;
    }

    if (telemetry)
    {
        telemetry->logSignal("ballistics:pulse_yaw", solution.yawAngle);
        telemetry->logSignal("ballistics:pulse_pitch", solution.pitchAngle);
        telemetry->logSignal("ballistics:pulse_tof", solution.timeOfFlight);
        telemetry->logSignal("ballistics:pulse_distance", solution.distance);
    }

    auto actualHitTimePosData = projectedAimPosData.projectForward(solution.timeOfFlight);

    aimAngle = atan2f(
        actualHitTimePosData.yPos - worldToTurret.getY(),
        actualHitTimePosData.xPos - worldToTurret.getX());

    // Calculate shot timing window
    // Use the active plate's actual radius for angular width calculation
    float plateAngularWidth = PLATE_WIDTH / activePlateRadius;

    // Calculate when the active plate's CENTER will actually cross the aim line
    // Plate i is at angle: theta + i*π/2
    float predictedActivePlateAngle = actualHitTimePosData.theta + activePlateIndex * M_PI_2;

    // Angle the target plate must move before reaching the aim line
    // We also handle the case where the plate does >+1 revolution
    // The `minDifference` is valid because if the plate selection works, then
    // `predictedActivePlateAngle` should be close to the aim line
    float totalActivePlateTravel = (actualHitTimePosData.theta - projectedAimPosData.theta) +
                                   Angle(predictedActivePlateAngle).minDifference(aimAngle + M_PI);

    float timeToPlateCenterCrossing = totalActivePlateTravel / omegaTotal;

    // Time for close edge to reach aim line
    float halfWidthTime = (plateAngularWidth / 2.0f) / fabsf(omegaTotal);
    float timeToCloseEdge = timeToPlateCenterCrossing - halfWidthTime;
    float timeToFarEdge = timeToPlateCenterCrossing + halfWidthTime;

    // Convert to absolute timestamps in microseconds
    // Shot window is when we should fire
    uint64_t currentTimeMicros = tap::arch::clock::getTimeMicroseconds();

    // We need to fire early enough that projectile arrives during plate crossing
    // Fire time = (plate crossing time) - (time of flight)
    float fireWindowStart = timeToCloseEdge - solution.timeOfFlight;
    float fireWindowEnd = timeToFarEdge - solution.timeOfFlight;

    solution.shotWindowStart = currentTimeMicros + static_cast<uint64_t>(fireWindowStart * 1e6f);
    solution.shotWindowEnd = currentTimeMicros + static_cast<uint64_t>(fireWindowEnd * 1e6f);

    if (telemetry)
    {
        // telemetry->logSignal("ballistics:pulse_angular_offset", angularOffset.getWrappedValue());
        telemetry->logSignal("ballistics:pulse_time_to_crossing", timeToPlateCenterCrossing);
        // telemetry->logSignal("ballistics:pulse_window_start_offset", startOffsetSeconds);
        // telemetry->logSignal("ballistics:pulse_window_end_offset", endOffsetSeconds);
        telemetry->logSignal(
            "ballistics:pulse_window_duration",
            solution.shotWindowEnd - solution.shotWindowStart);
    }

    return solution;
}
}  // namespace aruwsrc::algorithms
