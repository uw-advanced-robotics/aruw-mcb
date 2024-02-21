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

#include "sentry_transform_adapter.hpp"

#include "sentry_transforms.hpp"
#include "modm/math/geometry/location_2d.hpp"

namespace aruwsrc::sentry
{
SentryTransformAdapter::SentryTransformAdapter(const SentryTransforms& transforms)
    : transforms(transforms)
{
}

modm::Location2D<float> SentryTransformAdapter::getCurrentLocation2D() const
{
    tap::algorithms::transforms::Transform worldToChassis = transforms.getWorldToChassis();
    return modm::Location2D(worldToChassis.getX(), worldToChassis.getY(), worldToChassis.getYaw());
}

modm::Vector2f SentryTransformAdapter::getCurrentVelocity2D() const
{
    return transforms.getChassisOdometry().getCurrentVelocity2D();
}

float SentryTransformAdapter::getYaw() const { return transforms.getWorldToChassis().getYaw(); }

uint32_t SentryTransformAdapter::getLastComputedOdometryTime() const
{
    return transforms.getChassisOdometry().getLastComputedOdometryTime();
}

tap::algorithms::CMSISMat<3,1> SentryTransformAdapter::getTurretLocation(int turretID) const
{
    tap::algorithms::transforms::Transform worldToTurret = transforms.getWorldToTurret(turretID);
    const float positionData[3 * 1] = {worldToTurret.getX(), worldToTurret.getY(), worldToTurret.getZ()};
    tap::algorithms::CMSISMat<3,1> positionInCMS (positionData);
    return positionInCMS;

}

tap::algorithms::CMSISMat<3,1> SentryTransformAdapter::getTurretOrientation(int turretID) const
{
    tap::algorithms::transforms::Transform worldToTurret = transforms.getWorldToTurret(turretID);
    const float positionData[3 * 1] = {worldToTurret.getRoll(), worldToTurret.getYaw(), worldToTurret.getPitch()};
    tap::algorithms::CMSISMat<3,1> positionInCMS (positionData);
    return positionInCMS;
};

};  // namespace aruwsrc::sentry
