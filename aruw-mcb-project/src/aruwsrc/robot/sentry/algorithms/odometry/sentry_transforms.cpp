/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "sentry_transforms.hpp"

using namespace tap::algorithms::odometry;
using namespace tap::algorithms::transforms;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::sentry::turret;

namespace aruwsrc::sentry::algorithms::odometry
{
SentryTransforms::SentryTransforms(
    const tap::algorithms::odometry::Odometry2DInterface& chassisOdometry,
    const YawTurretSubsystem& turretMajor,
    const SentryTurretMinorSubsystem& turretLeft,
    const SentryTurretMinorSubsystem& turretRight,
    const SentryTransforms::SentryTransformConfig& config)
    : config(config),
      chassisOdometry(chassisOdometry),
      turretMajor(turretMajor),
      turretLeft(turretLeft),
      turretRight(turretRight),
      worldToChassis(Transform::identity()),
      worldToTurretMajor(Transform::identity()),
      worldToTurretLeft(Transform::identity()),
      worldToTurretRight(Transform::identity()),
      worldToVTM(Transform::identity()),
      chassisToArducam0(Transform::identity()),
      chassisToArducam1(Transform::identity()),
      chassisToArducam2(Transform::identity()),
      chassisToArducam3(Transform::identity()),
      chassisToTurretMajor(Transform::identity()),
      turretMajorToTurretLeft(0., config.turretMinorOffset, 0., 0., 0., 0.),
      turretMajorToTurretRight(0., -config.turretMinorOffset, 0., 0., 0., 0.)
{
}

void SentryTransforms::updateTransforms()
{
    // pose of chassis in world frame
    modm::Location2D chassisPose = chassisOdometry.getCurrentLocation2D();
    worldToChassis.updateTranslation(chassisPose.getX(), chassisPose.getY(), 0.);
    worldToChassis.updateRotation(0., 0., chassisPose.getOrientation());

    // Chassis to Turret Major
    chassisToTurretMajor.updateRotation(0., 0., turretMajor.getChassisYaw());

    // Turret Major to Minors
    turretMajorToTurretLeft.updateRotation(
        0.,
        turretLeft.pitchMotor.getChassisFrameMeasuredAngle().getWrappedValue(),
        turretLeft.yawMotor.getChassisFrameMeasuredAngle().getWrappedValue());
    turretMajorToTurretRight.updateRotation(
        0.,
        turretRight.pitchMotor.getChassisFrameMeasuredAngle().getWrappedValue(),
        turretRight.yawMotor.getChassisFrameMeasuredAngle().getWrappedValue());

    // World transforms
    worldToTurretMajor = worldToChassis.composeStatic(chassisToTurretMajor);

    worldToTurretLeft = worldToTurretMajor.composeStatic(turretMajorToTurretLeft);
    worldToTurretRight = worldToTurretMajor.composeStatic(turretMajorToTurretRight);
    worldToVTM = worldToTurretMajor;

    // Chassis to Arducam
    chassisToArducam0 = chassisToTurretMajor.composeStatic(MAJOR_TO_ARDUCAM1);
    chassisToArducam1 = chassisToTurretMajor.composeStatic(MAJOR_TO_ARDUCAM2);
    chassisToArducam2 = chassisToTurretMajor.composeStatic(MAJOR_TO_ARDUCAM3);
    chassisToArducam3 = chassisToTurretMajor.composeStatic(MAJOR_TO_ARDUCAM4);
}

}  // namespace aruwsrc::sentry::algorithms::odometry
