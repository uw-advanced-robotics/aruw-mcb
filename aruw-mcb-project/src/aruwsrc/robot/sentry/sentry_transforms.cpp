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

using namespace tap::algorithms::transforms;
using namespace aruwsrc::control::turret;
using namespace tap::algorithms::odometry;
using namespace aruwsrc::control::sentry;

namespace aruwsrc::sentry
{
SentryTransforms::SentryTransforms(
    const tap::algorithms::odometry::Odometry2DInterface& chassisOdometry,
    const YawTurretSubsystem& turretMajor,
    const SentryTurretMinorSubsystem& turretMinorGirlboss,
    const SentryTurretMinorSubsystem& turretMinorMalewife,
    const SentryTransforms::SentryTransformConfig& config)
    : config(config),
      chassisOdometry(chassisOdometry),
      turretMajor(turretMajor),
      turretMinorGirlboss(turretMinorGirlboss),
      turretMinorMalewife(turretMinorMalewife),
      worldToChassis(Transform::identity()),
      worldToTurretMajor(Transform::identity()),
      worldToTurretLeft(Transform::identity()),
      worldToTurretRight(Transform::identity()),
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
        turretMinorGirlboss.pitchMotor.getAngleFromCenter(),
        turretMinorGirlboss.yawMotor.getAngleFromCenter());
    turretMajorToTurretRight.updateRotation(
        0.,
        turretMinorMalewife.pitchMotor.getAngleFromCenter(),
        turretMinorMalewife.yawMotor.getAngleFromCenter());

    // World transforms
    worldToTurretMajor = worldToChassis.compose(chassisToTurretMajor);
    worldToTurretLeft = worldToTurretMajor.compose(turretMajorToTurretLeft);
    worldToTurretRight = worldToTurretMajor.compose(turretMajorToTurretRight);
}

}  // namespace aruwsrc::sentry
