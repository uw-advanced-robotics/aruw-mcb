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
    const Config config,
    const uint8_t turretID,
    aruwsrc::communication::rtt::RttTelemetry* telemetry)
    : visionCoprocessor(visionCoprocessor),
      transformer(transformer),
      worldToTurret(transformer.getWorldToTurret(turretID)),
      frictionWheels(frictionWheels),
      config(config),
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
        launchSpeed = config.defaultLaunchSpeed;
    }

    if (telemetry)
    {
        telemetry->logSignal("ballistics:launch_speed", launchSpeed);
    }

    // time in microseconds to project the target position ahead by
    int64_t aimDataAge = static_cast<int64_t>(tap::arch::clock::getTimeMicroseconds()) -
                         static_cast<int64_t>(aimData.timestamp);

    // project the target position forward in time s.t. we are computing a ballistics
    // solution for a target "now" rather than whenever the camera saw the target
    aruwsrc::communication::serial::VisionCoprocessor::PositionData targetDataNow =
        aimData.pva.projectForward(aimDataAge / 1E6f);

    if (telemetry)
    {
        telemetry->logSignal(
            "ballistics:target_pos",
            targetDataNow.xPos,
            targetDataNow.yPos,
            targetDataNow.zPos);
        telemetry->logSignal(
            "ballistics:target_vel",
            targetDataNow.xVel,
            targetDataNow.yVel,
            targetDataNow.zVel);
        telemetry->logSignal("ballistics:omega", targetDataNow.omega);
        telemetry->logSignal("ballistics:theta", targetDataNow.theta);
    }

    // TODO: make nicer
    omegaLP = omegaLPAlpha * targetDataNow.omega + (1 - omegaLPAlpha) * omegaLP;

    // Use enemy angular velocity to determine which aiming strategy to use
    // TODO: this could technically be the angular velocity in the rotating target-tracking
    // frame ("omegaTotal")
    if (fabsf(omegaLP) < config.shotTimingExitThreshold)
    {
        aimStrategy = AimStrategy::JITTER;
    }
    else if (fabsf(omegaLP) > config.shotTimingEntryThreshold)
    {
        // Use pulse estimation for fast rotating targets
        aimStrategy = AimStrategy::SHOT_GATING;
    }

    switch (aimStrategy)
    {
        case AimStrategy::JITTER:
        {
            communication::serial::VisionCoprocessor::PositionData targetDataLaunchTime =
                targetDataNow.projectForward(config.minimumShotDelay);

            lastComputedSolution = computeJitterAim(targetDataLaunchTime, launchSpeed);
            break;
        }

        case AimStrategy::SHOT_GATING:
        {
            lastComputedSolution = computePulseEstimation(targetDataNow, launchSpeed);
            break;
        }
    }

    return lastComputedSolution;
}

std::optional<CvBallisticsSolver::BallisticsSolution> CvBallisticsSolver::computeJitterAim(
    const communication::serial::VisionCoprocessor::PositionData& targetData,
    float launchSpeed)
{
    // Is our last targeted plate still valid?
    if (lastComputedSolution.has_value())
    {
        const uint8_t activePlate = lastComputedSolution->activePlateIndex;
        const float lastRadius = (activePlate % 2 == 0) ? targetData.radius0 : targetData.radius1;
        const float currTheta = targetData.theta + M_PI_2 * activePlate;

        RobotTargetKinematicState ballisticsTargetState(
            {targetData.xPos + lastRadius * cos(currTheta) - worldToTurret.getX(),
             targetData.yPos + lastRadius * sin(currTheta) - worldToTurret.getY(),
             targetData.zPos + targetData.plateHeights[activePlate] - worldToTurret.getZ()},
            {targetData.xVel - worldToTurret.getXVel(),
             targetData.yVel - worldToTurret.getYVel(),
             targetData.zVel},
            {targetData.xAcc, targetData.yAcc, targetData.zAcc},
            lastRadius,
            currTheta,
            targetData.omega);

        BallisticsSolution solution = BallisticsSolution();
        solution.distance = ballisticsTargetState.position.getLength();
        solution.usePulseEstimation = false;
        solution.activePlateIndex = activePlate;
        solution.shotWindowStart = 0;
        solution.shotWindowEnd = 0;

        if (ballistics::findTargetProjectileIntersection(  // ballistics has a solution
                ballisticsTargetState,
                launchSpeed,
                NUM_FORWARD_KINEMATIC_PROJECTIONS,
                &solution.pitchAngle,
                &solution.yawAngle,
                &solution.timeOfFlight,
                config.turretPitchOffset) &&
            compareFloatClose(  // plate will be facing us at time of impact
                Angle(
                    targetData.projectForward(solution.timeOfFlight).theta + M_PI_2 * activePlate +
                    M_PI)
                    .minDifference(solution.yawAngle),
                0,
                M_PI_4))
        {
            return solution;
        }
    }

    // Our last targeted plate is invalid, so we reselect
    std::optional<CvBallisticsSolver::BallisticsSolution> solution = std::nullopt;
    for (int i = 0; i < 4; i++)
    {
        float currRadius = (i % 2 == 0) ? targetData.radius0 : targetData.radius1;
        float currTheta = targetData.theta + M_PI_2 * i;

        RobotTargetKinematicState targetState(
            {targetData.xPos + currRadius * cos(currTheta) - worldToTurret.getX(),
             targetData.yPos + currRadius * sin(currTheta) - worldToTurret.getY(),
             targetData.zPos + targetData.plateHeights[i] - worldToTurret.getZ()},
            {targetData.xVel - worldToTurret.getXVel(),
             targetData.yVel - worldToTurret.getYVel(),
             targetData.zVel},
            {targetData.xAcc, targetData.yAcc, targetData.zAcc},
            currRadius,
            currTheta,
            targetData.omega);

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
                config.turretPitchOffset) &&
            (!solution || currentSolution.timeOfFlight < solution->timeOfFlight))
        {
            solution = currentSolution;
            if (telemetry)
            {
                telemetry->logSignal("ballistics:jitter_best_plate", static_cast<int>(i));
                telemetry->logSignal("ballistics:jitter_tof", currentSolution.timeOfFlight);
            }
        }
    }

    if (telemetry && solution)
    {
        telemetry->logSignal("ballistics:solution_found", true);
        telemetry->logSignal("ballistics:yaw", lastComputedSolution->yawAngle);
        telemetry->logSignal("ballistics:pitch", lastComputedSolution->pitchAngle);
        telemetry->logSignal("ballistics:distance", lastComputedSolution->distance);
    }
    return solution;
}

std::optional<CvBallisticsSolver::BallisticsSolution> CvBallisticsSolver::computePulseEstimation(
    const communication::serial::VisionCoprocessor::PositionData& targetData,
    float launchSpeed)
{
    // Pulse Estimation:
    // 1. Estimate ToF using simple distance/launch_speed calculation
    // 2. Compute omega_total accounting for both rotation and translation
    // 3. Determine which plate will be "active" (shootable) at time ToF
    // 4. Compute accurate ballistics solution for active plate
    // 5. Calculate shot timing window accounting for plate width and omega_total

    // Estimate approximate distance and ToF to nearest point on robot perimeter
    float avgRadius = (targetData.radius0 + targetData.radius1) / 2.0f;

    modm::Vector3f robotPos(
        targetData.xPos - worldToTurret.getX(),
        targetData.yPos - worldToTurret.getY(),
        targetData.zPos - worldToTurret.getZ());

    modm::Vector3f robotVel(
        targetData.xVel - worldToTurret.getXVel(),
        targetData.yVel - worldToTurret.getYVel(),
        targetData.zVel);

    float horizontalDistToClosestPoint = robotPos.xy().getLength() - avgRadius;
    float approxDistance = modm::Vector2f(horizontalDistToClosestPoint, robotPos.z).getLength();
    float estimatedToF = approxDistance / launchSpeed + config.minimumShotDelay;
    // TODO: could do a center ballistics pass instead? would account for turret pitch

    auto estHitTimeTargetData = targetData.projectForward(estimatedToF);

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
    float omegaTotal = targetData.omega + omegaFromTranslation;

    if (telemetry)
    {
        telemetry->logSignal("ballistics:pulse_omega_robot", targetData.omega);
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
        estHitTimeTargetData.yPos - worldToTurret.getY(),
        estHitTimeTargetData.xPos - worldToTurret.getX());

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
        WrappedFloat(estHitTimeTargetData.theta, 0, M_TWOPI) - aimAngle + M_PI;
    float desiredPlateQuadrantStartToProjectedPlate0 =
        (Angle(-desiredPlateQuadrantStart) - aimLineToProjectedPlate0).getWrappedValue();
    uint8_t activePlateIndex =
        static_cast<uint8_t>(desiredPlateQuadrantStartToProjectedPlate0 / M_PI_2);

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
    float activePlateHeight = targetData.plateHeights[activePlateIndex];
    float activePlateRadius = (activePlateIndex % 2 == 0) ? targetData.radius0 : targetData.radius1;

    // Robot center's current position, velocity, acceleration
    SecondOrderKinematicState robotCenterState(
        {targetData.xPos - worldToTurret.getX(),
         targetData.yPos - worldToTurret.getY(),
         targetData.zPos + activePlateHeight - worldToTurret.getZ()},
        {targetData.xVel - worldToTurret.getXVel(),
         targetData.yVel - worldToTurret.getYVel(),
         targetData.zVel},
        {targetData.xAcc, targetData.yAcc, targetData.zAcc});

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
            config.turretPitchOffset -
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

    auto actualHitTimeTargetData = targetData.projectForward(solution.timeOfFlight);

    aimAngle = atan2f(
        actualHitTimeTargetData.yPos - worldToTurret.getY(),
        actualHitTimeTargetData.xPos - worldToTurret.getX());

    // Calculate shot timing window
    // Use the active plate's actual radius for angular width calculation
    float plateAngularWidth = PLATE_WIDTH / activePlateRadius;

    // Calculate when the active plate's CENTER will actually cross the aim line
    // Plate i is at angle: theta + i*π/2
    float predictedActivePlateAngle = actualHitTimeTargetData.theta + activePlateIndex * M_PI_2;

    // Time until a shot we take will hit the plate center
    float timeToPlateCenterShot =
        Angle(predictedActivePlateAngle).minDifference(aimAngle + M_PI) / omegaTotal;

    // Time it takes for half the plate to cross the aim line
    float halfWidthTime = (plateAngularWidth / 2.0f) / fabsf(omegaTotal);

    // Window of time we should fire in to hit shots
    float shotWindowStart = timeToPlateCenterShot - halfWidthTime;
    float shotWindowEnd = timeToPlateCenterShot + halfWidthTime;

    // Convert to absolute timestamps in microseconds
    uint64_t currentTimeMicros = tap::arch::clock::getTimeMicroseconds();
    solution.shotWindowStart = currentTimeMicros + static_cast<uint64_t>(shotWindowStart * 1e6f);
    solution.shotWindowEnd = currentTimeMicros + static_cast<uint64_t>(shotWindowEnd * 1e6f);

    if (telemetry)
    {
        telemetry->logSignal("ballistics:pulse_angular_offset", predictedActivePlateAngle);
        telemetry->logSignal("ballistics:pulse_time_to_crossing", timeToPlateCenterShot);
        telemetry->logSignal("ballistics:pulse_window_start_offset", shotWindowStart);
        telemetry->logSignal("ballistics:pulse_window_end_offset", shotWindowEnd);
        telemetry->logSignal(
            "ballistics:pulse_window_duration",
            solution.shotWindowEnd - solution.shotWindowStart);
    }

    return solution;
}
}  // namespace aruwsrc::algorithms
