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

using namespace tap::algorithms::odometry;
using namespace tap::algorithms::transforms;
using namespace aruwsrc::algorithms::state;
using namespace aruwsrc::control::client_display;
using namespace aruwsrc::control::turret;

namespace aruwsrc::algorithms::transforms
{
StandardAndHeroTransformer::StandardAndHeroTransformer(
    const Odometry2DInterface& chassisOdometry,
    const OrientationObserverInterface<Frame::WORLD, Frame::CHASSIS>& chassisOrientationObserver,
    const OrientationObserverInterface<Frame::CHASSIS, Frame::TURRET>& turretEncoders,
    const OrientationObserverInterface<Frame::WORLD, Frame::TURRET>& turretImu,
    const tap::algorithms::transforms::Position& chassisToTurretTranslation)
    : chassisOdometry(chassisOdometry),
      chassisOrientationObserver(chassisOrientationObserver),
      turretEncoders(turretEncoders),
      turretImu(turretImu),
      worldToChassis(Transform::identity()),
      worldToTurret(Transform::identity()),
      chassisToTurret(Transform(
          chassisToTurretTranslation,
          Orientation(0, 0, 0))),  // do we care about z offset?
      worldToVTM(Transform::identity())
{
}

void StandardAndHeroTransformer::updateTransforms()
{
    modm::Location2D chassisPose = chassisOdometry.getCurrentLocation2D();
    worldToChassis.updateTranslation(chassisPose.getX(), chassisPose.getY(), 0.);

    // @note: here we are assuming that the chassis does not pitch or roll
    // This is fine for flat fields, but for an RMUC field with inclines
    // the state of the robot will not be properly tracked
    // worldToChassis.updateRotation(0., 0., chassisPose.getOrientation());
    DynamicOrientation chassisOrientation = chassisOrientationObserver.getOrientation();
    worldToChassis.updateRotation(chassisOrientation);

    if (turretImu.isOnline())
    {
        chassisToTurret.updateRotation(
            chassisOrientation.inverse().compose(turretImu.getOrientation()));
    }
    else
    {
        chassisToTurret.updateRotation(turretEncoders.getOrientation());
    }

    worldToTurret = worldToChassis.compose(chassisToTurret);

    worldToVTM = worldToTurret.composeStatic(VTM_OFFSET);
}

}  // namespace aruwsrc::algorithms::transforms
