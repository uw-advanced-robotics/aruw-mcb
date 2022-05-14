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

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/ballistics.hpp"
#include "tap/algorithms/odometry/odometry_2d_interface.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "aruwsrc/control/launcher/launch_speed_predictor_interface.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/drivers.hpp"

using namespace tap::algorithms;
using namespace modm;

namespace aruwsrc::algorithms
{
OttoBallisticsSolver::OttoBallisticsSolver(
    const Drivers &drivers,
    const tap::algorithms::odometry::Odometry2DInterface &odometryInterface,
    const control::turret::TurretSubsystem &turretSubsystem,
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
    turretOrigin = modm::Vector3f(aruwsrc::control::turret::TURRET_ORIGIN_RELATIVE_TO_REALSENSE[turretID]);
}

bool OttoBallisticsSolver::computeTurretAimAngles(
    float *pitchAngle,
    float *yawAngle,
    float *distance)
{
    const auto &aimData = drivers.visionCoprocessor.getLastAimData(turretID);

    // Verify that CV is actually online and that the aimData had a target
    if (!drivers.visionCoprocessor.isCvOnline() || !aimData.hasTarget)
    {
        return false;
    }

    // if the friction wheel launch speed is 0, use a default launch speed so ballistics gives a
    // reasonable computation
    const float launchSpeed = compareFloatClose(frictionWheels.getPredictedLaunchSpeed(), 0, 1E-5)
                                  ? defaultLaunchSpeed
                                  : frictionWheels.getPredictedLaunchSpeed();

    rotateVector(&turretOrigin, {.yaw = odometryInterface.getYaw()});
    const modm::Vector3f turretPosition = modm::Vector3f(odometryInterface.getCurrentLocation2D().getPosition(), 0) + turretOrigin;

    const Vector2f chassisVelocity = odometryInterface.getCurrentVelocity2D();

    // target state, frame whose axis is at the turret center and z is up
    // assume acceleration of the chassis is 0 since we don't measure it
    ballistics::MeasuredKinematicState targetState = {
        .position = {aimData.xPos - turretPosition.x, aimData.yPos - turretPosition.y, aimData.zPos - turretPosition.z},
        .velocity =
            {aimData.xVel - chassisVelocity.x, aimData.yVel - chassisVelocity.y, aimData.zVel},
        .acceleration = {aimData.xAcc, aimData.yAcc, aimData.zAcc},  // TODO consider using chassis
                                                                     // acceleration from IMU
    };

    uint32_t projectforwardtimedt = tap::arch::clock::getTimeMicroseconds() - aimData.timestamp;

    targetState.position = targetState.projectForward(projectforwardtimedt / 1E6f);

    *distance = targetState.position.getLength();

    return ballistics::findTargetProjectileIntersection(
        targetState,
        launchSpeed,
        3,
        pitchAngle,
        yawAngle);
}
}  // namespace aruwsrc::algorithms
