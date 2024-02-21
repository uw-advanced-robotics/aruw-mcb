/*
 * Copyright (c) 2020-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TRANSFORMER_INTERFACE
#define TRANSFORMER_INTERFACE

#include "modm/math/geometry/location_2d.hpp"

#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/algorithms/cmsis_mat.hpp"

namespace aruwsrc::algorithms::transforms
{
class TransformerInterface : public tap::algorithms::odometry::Odometry2DInterface
{
public:
    /**
     * @return The current location (x and y coordinate) and orientation (in radians).
     */
    virtual modm::Location2D<float> getCurrentLocation2D() const = 0;

    /**
     * @return The current x and y velocity (in m/s).
     */
    virtual modm::Vector2f getCurrentVelocity2D() const = 0;

    /**
     * @return The current yaw orientation of the chassis in the world frame in radians.
     */
    virtual float getYaw() const = 0;

    /**
     * @return The last time that odometry was computed (in microseconds).
     */
    virtual uint32_t getLastComputedOdometryTime() const = 0;

    virtual tap::algorithms::CMSISMat<3,1> getTurretLocation(int turretID) const = 0;

    virtual tap::algorithms::CMSISMat<3,1> getTurretOrientation(int turretID) const = 0;
};

}  // namespace aruwsrc::algorithms::transforms

#endif  // TRANSFORMER_INTERFACE
