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

#include "engineer_transforms.hpp"

using namespace tap::algorithms::odometry;
using namespace tap::algorithms::transforms;

namespace aruwsrc::algorithms::odometry
{
EngineerTransforms::EngineerTransforms(
    const tap::algorithms::odometry::Odometry2DInterface& chassisOdometry,
    const EngineerTransforms::EngineerTransformConfig& config)
    : config(config),
      chassisOdometry(chassisOdometry),
      worldToChassis(Transform::identity()),
      chassisToArducam(Transform::identity())
{
}

void EngineerTransforms::updateTransforms()
{
    // pose of chassis in world frame
    modm::Location2D chassisPose = chassisOdometry.getCurrentLocation2D();
    worldToChassis.updateTranslation(chassisPose.getX(), chassisPose.getY(), 0.);
    worldToChassis.updateRotation(0., 0., chassisPose.getOrientation());

    // Chassis to Arducam
    // chassisToArducam = chassisToTurretMajor.compose(ARDUCAM_OFFSET);
}

}  // namespace aruwsrc::algorithms::odometry
