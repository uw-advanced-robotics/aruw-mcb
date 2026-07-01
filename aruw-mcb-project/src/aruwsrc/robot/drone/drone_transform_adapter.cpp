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

#include "drone_transform_adapter.hpp"

#include "tap/architecture/clock.hpp"

namespace aruwsrc::drone
{
DroneTransformAdapter::DroneTransformAdapter(const DroneTransformer& transforms)
    : transforms(transforms)
{
}

modm::Vector2f DroneTransformAdapter::getChassisVelocity2d() const
{
    return modm::Vector2f(0.0f, 0.0f);
}

uint32_t DroneTransformAdapter::getLastComputedOdometryTime() const
{
    return tap::arch::clock::getTimeMicroseconds();
}

const tap::algorithms::transforms::Transform& DroneTransformAdapter::getWorldToChassis() const
{
    return transforms.getWorldToChassis();
}

const tap::algorithms::transforms::Transform& DroneTransformAdapter::getWorldToTurret(uint8_t) const
{
    return transforms.getWorldToTurret();
}

const tap::algorithms::transforms::Transform& DroneTransformAdapter::getWorldToVTM() const
{
    return transforms.getWorldToVTM();
}

const tap::algorithms::transforms::Transform& DroneTransformAdapter::getChassisToArducam(
    uint8_t) const
{
    return transforms.getChassisToArducam();
}
}  // namespace aruwsrc::drone
