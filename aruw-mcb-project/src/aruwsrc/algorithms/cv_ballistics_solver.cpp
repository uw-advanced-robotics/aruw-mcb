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
CvBallisticsSolver::CvBallisticsSolver(
    const aruwsrc::communication::serial::VisionCoprocessor &visionCoprocessor,
    const tap::algorithms::odometry::Odometry2DInterface &odometryInterface,
    const control::turret::RobotTurretSubsystem &turretSubsystem,
    const control::launcher::LaunchSpeedPredictorInterface &frictionWheels,
    const float defaultLaunchSpeed,
    const uint8_t turretID,
    aruwsrc::communication::rtt::RttTelemetry* telemetry)
    : visionCoprocessor(visionCoprocessor),
      odometryInterface(odometryInterface),
      turretSubsystem(turretSubsystem),
      frictionWheels(frictionWheels),
      defaultLaunchSpeed(defaultLaunchSpeed),
      turretID(turretID),
      telemetry(telemetry)
{
}

std::optional<CvBallisticsSolver::BallisticsSolution> CvBallisticsSolver::
    computeTurretAimAngles()
{
    const auto &aimData = visionCoprocessor.getLastAimData(turretID);

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

    if (lastAimDataTimestamp != aimData.timestamp ||
        lastOdometryTimestamp != odometryInterface.getLastComputedOdometryTime())
    {
        lastAimDataTimestamp = aimData.timestamp;
        lastOdometryTimestamp = odometryInterface.getLastComputedOdometryTime();

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

        // time in microseconds to project the target position ahead by
        int64_t projectForwardTimeDt =
            static_cast<int64_t>(tap::arch::clock::getTimeMicroseconds()) -
            static_cast<int64_t>(aimData.timestamp);

        // project the target position forward in time s.t. we are computing a ballistics
        // solution for a target "now" rather than whenever the camera saw the target
        auto projectedAimPosData = aimData.pva.projectForward(projectForwardTimeDt / 1E6f);
        
        if (telemetry)
        {
            telemetry->logSignal("ballistics:target_pos", projectedAimPosData.xPos, projectedAimPosData.yPos, projectedAimPosData.zPos);
            telemetry->logSignal("ballistics:target_vel", projectedAimPosData.xVel, projectedAimPosData.yVel, projectedAimPosData.zVel);
            telemetry->logSignal("ballistics:omega", projectedAimPosData.omega);
            telemetry->logSignal("ballistics:theta", projectedAimPosData.theta);
        }

        // Check omega threshold to determine which aiming strategy to use
        if (fabsf(projectedAimPosData.omega) < OMEGA_THRESHOLD)
        {
            lastComputedSolution = std::nullopt;
            for (int i = 0; i < 4; i++)
            {
                float currRadius = (i % 2 == 0) ? projectedAimPosData.rad0 : projectedAimPosData.rad1;
                float currTheta = projectedAimPosData.theta + M_PI_2 * i;

                aruwsrc::communication::serial::VisionCoprocessor::RobotOrbitKinematicState targetState(
                    {projectedAimPosData.xPos + currRadius * cos(currTheta) - turretPosition.x,
                     projectedAimPosData.yPos + currRadius * sin(currTheta) - turretPosition.y,
                     projectedAimPosData.zPos + projectedAimPosData.plateHeights[i] - turretPosition.z},
                    {projectedAimPosData.xVel - chassisVel.x,
                     projectedAimPosData.yVel - chassisVel.y,
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
                        turretSubsystem.getPitchOffset()) &&
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
            // Use pulse estimation for fast rotating targets
            
            // Check if we already have a valid pulse estimation solution with an open fire window
            uint64_t currentTimeMicros = tap::arch::clock::getTimeMicroseconds();
            bool hasValidPulseSolution = lastComputedSolution.has_value() && 
                                         lastComputedSolution->usePulseEstimation &&
                                         currentTimeMicros <= lastComputedSolution->shotWindowEnd;
            
            // Discard pulse solution if omega has dropped below threshold
            bool omegaBelowThreshold = fabsf(projectedAimPosData.omega) < OMEGA_THRESHOLD;
            
            if (hasValidPulseSolution && !omegaBelowThreshold)
            {
                // Keep existing solution - fire window is still open and omega still high
                // Don't recalculate, this prevents constantly changing target plates
                if (telemetry)
                {
                    uint64_t timeRemaining = lastComputedSolution->shotWindowEnd - currentTimeMicros;
                    telemetry->logSignal("ballistics:pulse_window_remaining_us", static_cast<float>(timeRemaining));
                }
                return lastComputedSolution;
            }
            else
            {
                // Either no existing solution, fire window closed, or omega dropped, so compute new solution
                lastComputedSolution = computePulseEstimation(
                    projectedAimPosData,
                    turretPosition,
                    chassisVel,
                    launchSpeed);
            }
        }
    }

    return lastComputedSolution;
}

std::optional<CvBallisticsSolver::BallisticsSolution> CvBallisticsSolver::
    computePulseEstimation(
        const communication::serial::VisionCoprocessor::PositionData& projectedAimPosData,
        const modm::Vector3f& turretPosition,
        const modm::Vector2f& chassisVel,
        float launchSpeed)
{
    // Pulse Estimation:
    // 1. Estimate ToF using simple distance/launch_speed calculation
    // 2. Compute omega_total accounting for both rotation and translation
    // 3. Determine which plate will be "active" (shootable) at time ToF
    // 4. Compute accurate ballistics solution for active plate (single pass)
    // 5. Calculate shot timing window accounting for plate width and omega_total

    // Estimate approximate distance and ToF to robot center
    float avgRadius = (projectedAimPosData.rad0 + projectedAimPosData.rad1) / 2.0f;
    modm::Vector3f robotCenterPos(
        projectedAimPosData.xPos + avgRadius * cos(projectedAimPosData.theta) - turretPosition.x,
        projectedAimPosData.yPos + avgRadius * sin(projectedAimPosData.theta) - turretPosition.y,
        projectedAimPosData.zPos - turretPosition.z);
    
    float approxDistance = robotCenterPos.getLength();
    float estimatedToF = approxDistance / launchSpeed;

    // Create state for computing omega_total
    modm::Vector3f robotPos3D(
        projectedAimPosData.xPos - turretPosition.x,
        projectedAimPosData.yPos - turretPosition.y,
        projectedAimPosData.zPos - turretPosition.z);
    modm::Vector3f robotVel3D(
        projectedAimPosData.xVel - chassisVel.x,
        projectedAimPosData.yVel - chassisVel.y,
        projectedAimPosData.zVel);

    // Create temporary state to use helper methods
    aruwsrc::communication::serial::VisionCoprocessor::RobotOrbitKinematicState tempState(
        robotPos3D,
        robotVel3D,
        {projectedAimPosData.xAcc, projectedAimPosData.yAcc, projectedAimPosData.zAcc},
        avgRadius,
        projectedAimPosData.theta,
        projectedAimPosData.omega);

    // Compute omega_total accounting for both rotation and translation
    float omegaTotal = tempState.computeOmegaTotal(robotPos3D, robotVel3D);
    
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

    // Calculate our aim angle (from turret to robot center)
    float aimAngle = atan2f(
        projectedAimPosData.yPos - turretPosition.y,
        projectedAimPosData.xPos - turretPosition.x);

    // Determine active plate based on omega_total and estimated ToF
    uint8_t activePlateIndex = tempState.determineActivePlate(
        omegaTotal, 
        estimatedToF, 
        PLATE_WIDTH,
        aimAngle,
        projectedAimPosData.theta);
    
    if (telemetry)
    {
        telemetry->logSignal("ballistics:pulse_active_plate", static_cast<int>(activePlateIndex));
        telemetry->logSignal("ballistics:pulse_estimated_tof", estimatedToF);
        telemetry->logSignal("ballistics:pulse_aim_angle", aimAngle);
    }

    // Aim at active plate's current position
    // The RobotOrbitKinematicState model will handle projecting both:
    // 1. Linear motion of robot center (constant acceleration)
    // 2. Rotational motion of the plate around the center (constant angular velocity)
    float activePlateHeight = projectedAimPosData.plateHeights[activePlateIndex];
    float activePlateRadius = (activePlateIndex % 2 == 0) ? projectedAimPosData.rad0 : projectedAimPosData.rad1;
    float activePlateTheta = projectedAimPosData.theta + activePlateIndex * M_PI_2;
    
    // Active plate's current position (robot center + rotational offset)
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
            turretSubsystem.getPitchOffset()))
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

    // Calculate shot timing window
    // Use the active plate's actual radius for angular width calculation
    float plateAngularWidth = PLATE_WIDTH / activePlateRadius;

    // Calculate when the active plate's CENTER will actually cross the aim line
    // Plate i is at angle: theta + i*π/2
    float activePlateAngle = projectedAimPosData.theta + activePlateIndex * M_PI_2;
    
    // Angular distance from plate to aim line
    float angularOffset = activePlateAngle - aimAngle;
    
    // Normalize to [-π, π]
    while (angularOffset > M_PI) angularOffset -= 2.0f * M_PI;
    while (angularOffset < -M_PI) angularOffset += 2.0f * M_PI;
    
    // Calculate time for plate center to cross aim line
    float timeToPlateCenterCrossing;
    if (omegaTotal > 0)
    {
        // Counterclockwise rotation
        if (angularOffset < 0)
        {
            // Plate is behind, add full rotation
            angularOffset += 2.0f * M_PI;
        }
        timeToPlateCenterCrossing = angularOffset / omegaTotal;
    }
    else
    {
        // Clockwise rotation
        if (angularOffset > 0)
        {
            // Plate is ahead, subtract full rotation
            angularOffset -= 2.0f * M_PI;
        }
        timeToPlateCenterCrossing = angularOffset / omegaTotal;  // Both negative
    }

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
    
    // Clamp to future times only (can't fire in the past)
    float startOffsetSeconds = (fireWindowStart > 0.0f) ? fireWindowStart : 0.0f;
    float endOffsetSeconds = (fireWindowEnd > startOffsetSeconds) ? fireWindowEnd : startOffsetSeconds;
    
    solution.shotWindowStart = currentTimeMicros + static_cast<uint64_t>(startOffsetSeconds * 1e6f);
    solution.shotWindowEnd = currentTimeMicros + static_cast<uint64_t>(endOffsetSeconds * 1e6f);
    
    if (telemetry)
    {
        telemetry->logSignal("ballistics:pulse_angular_offset", angularOffset);
        telemetry->logSignal("ballistics:pulse_time_to_crossing", timeToPlateCenterCrossing);
        telemetry->logSignal("ballistics:pulse_window_start_offset", startOffsetSeconds);
        telemetry->logSignal("ballistics:pulse_window_end_offset", endOffsetSeconds);
        telemetry->logSignal("ballistics:pulse_window_duration", endOffsetSeconds - startOffsetSeconds);
    }

    return solution;
}
}  // namespace aruwsrc::algorithms
