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

#include "modm/math/geometry/location_2d.hpp"

#include "sentry_transforms.hpp"

using namespace tap::algorithms::transforms;

namespace aruwsrc::sentry
{
SentryTransformAdapter::SentryTransformAdapter(const SentryTransforms& transforms)
    : transforms(transforms)
{
}

modm::Vector2f SentryTransformAdapter::getChassisVelocity2d() const
{
    return transforms.getChassisOdometry().getCurrentVelocity2D();
}

uint32_t SentryTransformAdapter::getLastComputedOdometryTime() const
{
    // @todo: move into transformer
    return transforms.getChassisOdometry().getLastComputedOdometryTime();
}

const Transform& SentryTransformAdapter::getWorldToChassis() const
{
    return this->transforms.getWorldToChassis();
}

const Transform& SentryTransformAdapter::getWorldToTurret(uint8_t turretID) const
{
    // todo: waiting for !646 to merge before can check ids correctly
    if (turretID == 0)
    {
        return this->transforms.getWorldToTurretLeft();
    }
    else
    {
        return this->transforms.getWorldToTurretLeft();
    }
}

};  // namespace aruwsrc::sentry
