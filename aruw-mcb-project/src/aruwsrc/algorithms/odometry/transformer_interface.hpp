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

#ifndef TRANSFORMER_INTERFACE_HPP_
#define TRANSFORMER_INTERFACE_HPP_

#include "tap/algorithms/cmsis_mat.hpp"
#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/algorithms/transforms/transform.hpp"

#include "modm/math/geometry/location_2d.hpp"

namespace aruwsrc::algorithms::transforms
{
class TransformerInterface
{
public:
    /**
     * @return The current x and y velocity of the chassis in the chassis frame (in m/s).
     */
    virtual modm::Vector2f getChassisVelocity2d() const = 0;

    /**
     * @return The last time that odometry was computed (in microseconds).
     */
    virtual uint32_t getLastComputedOdometryTime() const = 0;

    virtual const tap::algorithms::transforms::Transform& getWorldToChassis() const = 0;

    virtual const tap::algorithms::transforms::Transform& getWorldToTurret(
        uint8_t turretID) const = 0;
};

}  // namespace aruwsrc::algorithms::transforms

#endif  // TRANSFORMER_INTERFACE
