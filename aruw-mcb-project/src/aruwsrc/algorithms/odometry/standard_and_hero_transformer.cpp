/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "standard_and_hero_transformer.hpp"

using namespace aruwsrc::control::turret;
using namespace tap::algorithms::odometry;
using namespace tap::algorithms::transforms;

namespace aruwsrc::algorithms::transforms
{
StandardAndHeroTransformer::StandardAndHeroTransformer(
    const Odometry2DInterface& chassisOdometry,
    const RobotTurretSubsystem& turret)
    : chassisOdometry(chassisOdometry),
      turret(turret),
      worldToChassis(Transform::identity()),
      worldToTurret(Transform::identity()),
      chassisToTurret(Transform::identity())  // do we care about z offset?
{
}

void StandardAndHeroTransformer::updateTransforms()
{
    modm::Location2D chassisPose = chassisOdometry.getCurrentLocation2D();
    worldToChassis.updateTranslation(chassisPose.getX(), chassisPose.getY(), 0.);
    worldToChassis.updateRotation(0., 0., chassisPose.getOrientation());

    worldToTurret.updateRotation(0.0f, turret.getWorldPitch(), turret.getWorldYaw());

    // turret is in center of chassis, for now no z offset?
    worldToTurret.updateTranslation(worldToChassis.getTranslation());

    chassisToTurret.updateRotation(
        worldToTurret.getRoll() - worldToChassis.getRoll(),
        worldToTurret.getPitch() - worldToChassis.getPitch(),
        worldToTurret.getYaw() - worldToChassis.getYaw());
}

}  // namespace aruwsrc::algorithms::transforms
