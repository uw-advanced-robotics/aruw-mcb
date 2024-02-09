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

#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "standard_and_hero_transformer.hpp"

namespace aruwsrc::algorithms::transforms
{

class StandardAndHeroTransformAdapter : public tap::algorithms::odometry::Odometry2DInterface
{
    StandardAndHeroTransformAdapter(
        const StandardAndHeroTransformer& transforms
    );

    virtual modm::Location2D<float> getCurrentLocation2D() const = 0;

    virtual modm::Vector2f getCurrentVelocity2D() const = 0;

    virtual float getYaw() const = 0;

    virtual uint32_t getLastComputedOdometryTime() const = 0;

    private:
        StandardAndHeroTransformer transforms;
};

}  // namespace aruwsrc::algorithms::transforms

#endif  // STANDARD_AND_HERO_TRANSFORM_ADAPTER_HPP_
