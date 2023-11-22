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

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"

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

    // @note: here we are assuming that the chassis does not pitch or roll
    // This is fine for flat fields, but for an RMUC field with inclines
    // the state of the robot will not be properly tracked
    worldToChassis.updateRotation(0., 0., chassisPose.getOrientation());

    float roll = 0.0f;
    const aruwsrc::can::TurretMCBCanComm* turretMCB = turret.getTurretMCB();

    // @note: must shift by PI / 3 since turret mcb roll is in the range [- PI / 2, PI / 2] while
    // turret pitch and yaw are both in [0, PI]
    if (turretMCB != nullptr) roll = turretMCB->getRoll() + M_PI_2;

    worldToTurret.updateRotation(roll, turret.getWorldPitch(), turret.getWorldYaw());

    worldToTurret.updateTranslation(worldToChassis.getTranslation());
    chassisToTurret = worldToChassis.getInverse().compose(worldToTurret);
}

}  // namespace aruwsrc::algorithms::transforms
