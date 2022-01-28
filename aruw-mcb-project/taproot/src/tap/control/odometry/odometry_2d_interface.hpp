/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef ODOMETRY_2D_INTERFACE_HPP_
#define ODOMETRY_2D_INTERFACE_HPP_

// Include instead of forward declare because template uses default arguments
#include "modm/math/geometry/location_2d.hpp"

namespace tap::control::odometry
{
/**
 * @brief Interface for retrieving the position of a robot chassis in 2 dimensions.
 *
 * The coordinates measured are relative to some world coordinate frame which has a
 * position and orientation. At startup the world coordinate frame's position and
 * orientation are implementation defined.
 *
 * The position or orientation of the world coordinate frame can be set to match
 * the chassis's position or orientation through
 * `Odometry2DInterface::resetWorldFramePosition` and
 * `Odometry2DInterface::resetWorldFrameOrientation` respectively.
 *
 * As to why this interface exists separately from a more generic 3D implementation:
 * 2D odometry requires less sensor data (all it needs is chassis yaw in the world
 * frame), and performs simpler computations than a 3D odometry system would require.
 */
class Odometry2DInterface
{
public:
    /**
     * @return the current location (x and y coordinate) and orientation (in radians)
     */
    virtual inline const modm::Location2D<float>& getCurrentLocation2D() const = 0;
};

}  // namespace tap::control::odometry

#endif  // ODOMETRY_2D_INTERFACE_HPP_
