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

#ifndef DRONE_TRANSFORM_ADAPTER_HPP_
#define DRONE_TRANSFORM_ADAPTER_HPP_

#include "aruwsrc/algorithms/odometry/transforms/transformer_interface.hpp"

#include "drone_transformer.hpp"

namespace aruwsrc::drone
{
class DroneTransformAdapter : public aruwsrc::algorithms::odometry::transforms::TransformerInterface
{
public:
    explicit DroneTransformAdapter(const DroneTransformer& transforms);

    modm::Vector2f getChassisVelocity2d() const override;
    uint32_t getLastComputedOdometryTime() const override;
    const tap::algorithms::transforms::Transform& getWorldToChassis() const override;
    const tap::algorithms::transforms::Transform& getWorldToTurret(uint8_t) const override;
    const tap::algorithms::transforms::Transform& getWorldToVTM() const override;
    const tap::algorithms::transforms::Transform& getChassisToArducam(uint8_t) const override;

private:
    const DroneTransformer& transforms;
};
}  // namespace aruwsrc::drone

#endif  // DRONE_TRANSFORM_ADAPTER_HPP_
