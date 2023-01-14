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

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/launcher/launch_speed_predictor_interface.hpp"
#include "aruwsrc/control/turret/turret_constants.hpp"
#include "aruwsrc/control/turret/robot_turret_subsystem.hpp"
#include "aruwsrc/drivers.hpp"

using namespace tap::algorithms;
using namespace modm;

namespace aruwsrc::algorithms
{
OttoBallisticsSolver::OttoBallisticsSolver(
    const Drivers &drivers,
    const tap::algorithms::odometry::Odometry2DInterface &odometryInterface,
    const control::turret::RobotTurretSubsystem &turretSubsystem,
    const control::launcher::LaunchSpeedPredictorInterface &frictionWheels,
    const float defaultLaunchSpeed,
    const uint8_t turretID)
    : drivers(drivers),
      odometryInterface(odometryInterface),
      turretSubsystem(turretSubsystem),
      frictionWheels(frictionWheels),
      defaultLaunchSpeed(defaultLaunchSpeed),
      turretID(turretID)
{
}

std::optional<OttoBallisticsSolver::BallisticsSolution> OttoBallisticsSolver::
    computeTurretAimAngles()
{
    const auto &aimData = drivers.visionCoprocessor.getLastAimData(turretID);
    // Verify that CV is actually online and that the aimData had a target
    if (!drivers.visionCoprocessor.isCvOnline() || !aimData.hasTarget)
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

        // target state, frame whose axis is at the turret center and z is up
        // assume acceleration of the chassis is 0 since we don't measure it
        ballistics::MeasuredKinematicState targetState = {
            .position =
                {aimData.xPos - turretPosition.x,
                 aimData.yPos - turretPosition.y,
                 aimData.zPos - turretPosition.z},
            .velocity = {aimData.xVel - chassisVel.x, aimData.yVel - chassisVel.y, aimData.zVel},
            .acceleration =
                {aimData.xAcc, aimData.yAcc, aimData.zAcc},  // TODO consider using chassis
                                                             // acceleration from IMU
        };

        // time in microseconds to project the target position ahead by
        int64_t projectForwardTimeDt =
            static_cast<int64_t>(tap::arch::clock::getTimeMicroseconds()) -
            static_cast<int64_t>(aimData.timestamp);

        // project the target position forward in time s.t. we are computing a ballistics solution
        // for a target "now" rather than whenever the camera saw the target
        targetState.position = targetState.projectForward(projectForwardTimeDt / 1E6f);

        lastComputedSolution = BallisticsSolution();
        lastComputedSolution->distance = targetState.position.getLength();

        if (!ballistics::findTargetProjectileIntersection(
                targetState,
                launchSpeed,
                3,
                &lastComputedSolution->pitchAngle,
                &lastComputedSolution->yawAngle,
                &lastComputedSolution->timeOfFlight,
                turretSubsystem.getPitchOffset()))
        {
            lastComputedSolution = std::nullopt;
        }
    }

    return lastComputedSolution;
}
}  // namespace aruwsrc::algorithms
