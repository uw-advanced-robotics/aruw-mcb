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

#ifndef STANDARD_AND_HERO_TRANSFORM_ADAPTER_HPP_
#define STANDARD_AND_HERO_TRANSFORM_ADAPTER_HPP_

#include "standard_and_hero_transformer.hpp"
#include "transformer_interface.hpp"
#include "tap/algorithms/cmsis_mat.hpp"

namespace aruwsrc::algorithms::transforms
{
class StandardAndHeroTransformAdapter : public TransformerInterface
{

    StandardAndHeroTransformAdapter(const StandardAndHeroTransformer& transforms);

    modm::Location2D<float> getCurrentLocation2D() const;

    modm::Vector2f getCurrentVelocity2D() const;

    float getYaw() const;

    uint32_t getLastComputedOdometryTime() const;

    tap::algorithms::CMSISMat<3,1> getTurretLocation(int turretID) const;

    tap::algorithms::CMSISMat<3,1> getTurretOrientation(int turretID) const;

private:
    StandardAndHeroTransformer& transforms;
};

}  // namespace aruwsrc::algorithms::transforms

#endif  // STANDARD_AND_HERO_TRANSFORM_ADAPTER_HPP_
