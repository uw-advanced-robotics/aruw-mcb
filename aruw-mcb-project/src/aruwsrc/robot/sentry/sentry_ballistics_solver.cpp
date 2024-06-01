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

namespace aruwsrc::sentry
{
SentryBallisticsSolver::SentryBallisticsSolver(
    const aruwsrc::serial::VisionCoprocessor &visionCoprocessor,
    const aruwsrc::sentry::SentryTransforms &transformer,
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

        lastComputedSolution = BallisticsSolution();
        lastComputedSolution->distance = targetState.position.getLength();

        if (!ballistics::findTargetProjectileIntersection(
                targetState,
                launchSpeed,
                3,
                &lastComputedSolution->pitchAngle,
                &lastComputedSolution->yawAngle,
                &lastComputedSolution->timeOfFlight,
                turretPitchOffset))
        {
            lastComputedSolution = std::nullopt;
        }
    }

    return lastComputedSolution;
}
}  // namespace aruwsrc::sentry
