/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef VISION_ODOMETRY_DATA_PROVIDER_HPP_
#define VISION_ODOMETRY_DATA_PROVIDER_HPP_

#include "modm/math/geometry/location_2d.hpp"

namespace aruwsrc::algorithms::odometry
{
class VisionOdometryDataProvider
{
public:
    virtual ~VisionOdometryDataProvider() = default;

    virtual modm::Location2D<float> getVisionCurrentLocation2D() const = 0;

    virtual modm::Vector2f getVisionCurrentVelocity2D() const = 0;

    virtual float getVisionYaw() const = 0;
};
}  // namespace aruwsrc::algorithms::odometry

#endif  // VISION_ODOMETRY_DATA_PROVIDER_HPP_
