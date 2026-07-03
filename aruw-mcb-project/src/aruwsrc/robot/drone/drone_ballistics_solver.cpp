/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "drone_ballistics_solver.hpp"

#include "tap/algorithms/ballistics.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"

#include "aruwsrc/control/launcher/launch_speed_predictor_interface.hpp"
#include "aruwsrc/control/turret/robot_turret_subsystem.hpp"
#include "aruwsrc/robot/drone/drone_transform_adapter.hpp"

using namespace tap::algorithms;
using namespace modm;

namespace aruwsrc::drone
{
DroneBallisticsSolver::DroneBallisticsSolver(
    const aruwsrc::communication::serial::VisionCoprocessor& visionCoprocessor,
    const DroneTransformAdapter& transformAdapter,
    const control::turret::RobotTurretSubsystem& turretSubsystem,
    const control::launcher::LaunchSpeedPredictorInterface& frictionWheels,
    const float defaultLaunchSpeed,
    const uint8_t turretID)
    : BallisticsSolverInterface(turretID),
      visionCoprocessor(visionCoprocessor),
      transformAdapter(transformAdapter),
      turretSubsystem(turretSubsystem),
      frictionWheels(frictionWheels),
      defaultLaunchSpeed(defaultLaunchSpeed)
{
}

std::optional<DroneBallisticsSolver::BallisticsSolution> DroneBallisticsSolver::
    computeTurretAimAngles()
{
    const auto& aimData = visionCoprocessor.getLastAimData(turretID);
    if (!visionCoprocessor.isCvOnline() || !aimData.pva.updated)
    {
        lastComputedSolution = std::nullopt;
        return std::nullopt;
    }

    const uint32_t transformTimestamp = transformAdapter.getLastComputedOdometryTime();
    if (lastAimDataTimestamp != aimData.timestamp || lastTransformTimestamp != transformTimestamp)
    {
        lastAimDataTimestamp = aimData.timestamp;
        lastTransformTimestamp = transformTimestamp;

        float launchSpeed = frictionWheels.getPredictedLaunchSpeed();
        if (compareFloatClose(launchSpeed, 0.0f, 1e-5f))
        {
            launchSpeed = defaultLaunchSpeed;
        }

        const auto& worldToTurret = transformAdapter.getWorldToTurret(turretID);
        const modm::Vector3f turretPosition(
            worldToTurret.getX(),
            worldToTurret.getY(),
            worldToTurret.getZ());

        const Vector2f chassisVel = transformAdapter.getChassisVelocity2d();

        ballistics::SecondOrderKinematicState targetState(
            modm::Vector3f(
                aimData.pva.xPos - turretPosition.x,
                aimData.pva.yPos - turretPosition.y,
                aimData.pva.zPos - turretPosition.z),
            modm::Vector3f(
                aimData.pva.xVel - chassisVel.x,
                aimData.pva.yVel - chassisVel.y,
                aimData.pva.zVel),
            modm::Vector3f(aimData.pva.xAcc, aimData.pva.yAcc, aimData.pva.zAcc));

        const int64_t projectForwardTimeDt =
            static_cast<int64_t>(tap::arch::clock::getTimeMicroseconds()) -
            static_cast<int64_t>(aimData.timestamp);

        targetState.position = targetState.projectForward(projectForwardTimeDt / 1E6f);

        lastComputedSolution = BallisticsSolution();
        lastComputedSolution->distance = targetState.position.getLength();

        if (!ballistics::findTargetProjectileIntersection(
                targetState,
                launchSpeed,
                NUM_FORWARD_KINEMATIC_PROJECTIONS,
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
}  // namespace aruwsrc::drone
