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

#include "tap/algorithms/cmsis_mat.hpp"

#include "standard_and_hero_transformer.hpp"
#include "transformer_interface.hpp"

using namespace tap::algorithms::transforms;
namespace aruwsrc::algorithms::transforms
{
StandardAndHeroTransformAdapter::StandardAndHeroTransformAdapter(
    const StandardAndHeroTransformer& transforms)
    : transforms(transforms)
{
}

modm::Vector2f StandardAndHeroTransformAdapter::getChassisVelocity2d() const
{
    return transforms.getChassisOdometry().getCurrentVelocity2D();
}

uint32_t StandardAndHeroTransformAdapter::getLastComputedOdometryTime() const
{
    return transforms.getChassisOdometry().getLastComputedOdometryTime();
}

const Transform& StandardAndHeroTransformAdapter::getWorldToChassis() const
{
    return transforms.getWorldToChassis();
}
const Transform& StandardAndHeroTransformAdapter::getWorldToTurret(uint8_t turretID) const
{
    turretID = turretID;  // for pipeline checks (unused params)
    return transforms.getWorldToTurret();
};

};  // namespace aruwsrc::algorithms::transforms
