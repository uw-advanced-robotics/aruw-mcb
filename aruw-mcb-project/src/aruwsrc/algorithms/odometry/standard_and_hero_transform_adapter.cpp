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

#include "transformer_interface.hpp"

#include "standard_and_hero_transformer.hpp"
#include "tap/algorithms/cmsis_mat.hpp"

namespace aruwsrc::algorithms::transforms
{
StandardAndHeroTransformAdapter::StandardAndHeroTransformAdapter(
    const StandardAndHeroTransformer& transforms)
    : transforms(transforms)
{
}

modm::Location2D<float> StandardAndHeroTransformAdapter::getCurrentLocation2D() const
{
    const tap::algorithms::transforms::Transform worldToChassis = transforms.getWorldToChassis();
    return modm::Location2D(worldToChassis.getX(), worldToChassis.getY(), worldToChassis.getYaw());
}

modm::Vector2f StandardAndHeroTransformAdapter::getCurrentVelocity2D() const
{
    return transforms.getChassisOdometry().getCurrentVelocity2D();
}

float StandardAndHeroTransformAdapter::getYaw() const
{
    return transforms.getWorldToChassis().getYaw();
}

uint32_t StandardAndHeroTransformAdapter::getLastComputedOdometryTime() const
{
    return transforms.getChassisOdometry().getLastComputedOdometryTime();
}

tap::algorithms::CMSISMat<3,1> StandardAndHeroTransformAdapter::getTurretLocation(int turretID) const
{
    //Irrelevant Parameter in standard, gets rid of warning
    turretID = turretID;
    tap::algorithms::transforms::Transform worldToTurret = transforms.getWorldToTurret();
    const float positionData[3 * 1] = {worldToTurret.getX(), worldToTurret.getY(), worldToTurret.getZ()};
    tap::algorithms::CMSISMat<3,1> positionInCMS (positionData);
    return positionInCMS;

}

tap::algorithms::CMSISMat<3,1> StandardAndHeroTransformAdapter::getTurretOrientation(int turretID) const
{
    //Irrelevant Parameter in standard, gets rid of warning
    turretID = turretID;
    tap::algorithms::transforms::Transform worldToTurret = transforms.getWorldToTurret();
    const float positionData[3 * 1] = {worldToTurret.getRoll(), worldToTurret.getYaw(), worldToTurret.getPitch()};
    tap::algorithms::CMSISMat<3,1> positionInCMS (positionData);
    return positionInCMS;
};

};  // namespace aruwsrc::algorithms::transforms
