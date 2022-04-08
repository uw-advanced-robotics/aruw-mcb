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
#include "tap/algorithms/odometry/odometry_2d_interface.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "aruwsrc/control/launcher/launch_speed_predictor_interface.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "aruwsrc/drivers.hpp"

using namespace tap::algorithms;
using namespace modm;

namespace aruwsrc::algorithms
{
OttoBallisticsSolver::OttoBallisticsSolver(
    const Drivers &drivers,
    const tap::algorithms::odometry::Odometry2DInterface &odometryInterface,
    const chassis::ChassisSubsystem &chassisSubsystem,
    const control::turret::TurretSubsystem &turretSubsystem,
    const control::launcher::LaunchSpeedPredictorInterface &frictionWheels,
    const float defaultLaunchSpeed)
    : drivers(drivers),
      odometryInterface(odometryInterface),
      chassisSubsystem(chassisSubsystem),
      turretSubsystem(turretSubsystem),
      frictionWheels(frictionWheels),
      defaultLaunchSpeed(defaultLaunchSpeed)
{
}

bool OttoBallisticsSolver::computeTurretAimAngles(float *pitchAngle, float *yawAngle)
{
    const auto &aimData = drivers.visionCoprocessor.getLastAimData();

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

    const Vector2f robotPosition = odometryInterface.getCurrentLocation2D().getPosition();

    Matrix<float, 3, 1> chassisVelocity = chassisSubsystem.getActualVelocityChassisRelative();
    const float worldRelativeOrientation =
        drivers.turretMCBCanComm.getYaw() - this->turretSubsystem.getYawAngleFromCenter();
    chassisSubsystem.getVelocityWorldRelative(chassisVelocity, toRadian(worldRelativeOrientation));

    // target state, frame whose axis is at the turret center and z is up
    // assume acceleration of the chassis is 0 since we don't measure it
    ballistics::MeasuredKinematicState targetState = {
        .position = {aimData.xPos - robotPosition.x, aimData.yPos - robotPosition.y, aimData.zPos},
        .velocity =
            {aimData.xVel - chassisVelocity[0][0],
             aimData.yVel - chassisVelocity[1][0],
             aimData.zVel},
        .acceleration = {aimData.xAcc, aimData.yAcc, aimData.zAcc},  // TODO consider using chassis
                                                                     // acceleration from IMU
    };

    uint32_t projectforwardtimedt = tap::arch::clock::getTimeMicroseconds() - aimData.timestamp;

    targetState.position = targetState.projectForward(projectforwardtimedt / 1E6f);

    return ballistics::findTargetProjectileIntersection(
        targetState,
        launchSpeed,
        3,
        pitchAngle,
        yawAngle);
}
}  // namespace aruwsrc::algorithms
