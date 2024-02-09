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

#include "standard_and_hero_transform_adapter.hpp"

#include "tap/algorithms/odometry/odometry_2d_interface.hpp"

#include "standard_and_hero_transformer.hpp"

namespace aruwsrc::algorithms::transforms
{
StandardAndHeroTransformAdapter::StandardAndHeroTransformAdapter(
    const StandardAndHeroTransformer& transforms)
    : transforms(transforms)
{
}

modm::Location2D<float> StandardAndHeroTransformAdapter::getCurrentLocation2D() const
{
    return transforms.getChassisOdometry().getCurrentLocation2D();
}

modm::Vector2f StandardAndHeroTransformAdapter::getCurrentVelocity2D() const
{
    return transforms.getChassisOdometry().getCurrentVelocity2D();
}

float StandardAndHeroTransformAdapter::getYaw() const
{
    return transforms.getChassisOdometry().getYaw();
}

uint32_t StandardAndHeroTransformAdapter::getLastComputedOdometryTime() const
{
    return transforms.getChassisOdometry().getLastComputedOdometryTime();
}

};  // namespace namespace aruwsrc::algorithms::transforms
