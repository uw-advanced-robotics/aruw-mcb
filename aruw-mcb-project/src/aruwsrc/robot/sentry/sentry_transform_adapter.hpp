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

#ifndef SENTRY_TRANSFORM_ADAPTER_HPP_
#define SENTRY_TRANSFORM_ADAPTER_HPP_

#include "aruwsrc/algorithms/odometry/transformer_interface.hpp"

#include "sentry_transforms.hpp"

namespace aruwsrc::sentry
{
class SentryTransformAdapter : public aruwsrc::algorithms::transforms::TransformerInterface
{
public:
    SentryTransformAdapter(const SentryTransforms& transforms);

    modm::Vector2f getChassisVelocity2d() const;

    uint32_t getLastComputedOdometryTime() const;

    const tap::algorithms::transforms::Transform& getWorldToChassis() const;

    const tap::algorithms::transforms::Transform& getWorldToTurret(uint8_t turretID) const;

    inline uint8_t getNumTurrets() const { return numTurrets; };

private:
    const SentryTransforms& transforms;
    const uint8_t numTurrets = 2;
};

}  // namespace aruwsrc::sentry

#endif  // SENTRY_TRANSFORM_ADAPTER_HPP_
