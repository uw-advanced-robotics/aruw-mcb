/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TRANSFORMER_INTERFACE_MOCK_HPP_
#define TRANSFORMER_INTERFACE_MOCK_HPP_

#include <gmock/gmock.h>

#include "tap/algorithms/transforms/transform.hpp"

#include "aruwsrc/algorithms/odometry/transformer_interface.hpp"

namespace aruwsrc::mock
{
class TransformerInterface : public aruwsrc::algorithms::transforms::TransformerInterface
{
public:
    MOCK_METHOD(modm::Vector2f, getChassisVelocity2d, (), (const override));
    MOCK_METHOD(uint32_t, getLastComputedOdometryTime, (), (const override));
    MOCK_METHOD(tap::algorithms::transforms::Transform&, getWorldToChassis, (), (const override));
    MOCK_METHOD(
        tap::algorithms::transforms::Transform&,
        getWorldToTurret,
        (uint8_t),
        (const override));
};
}  // namespace aruwsrc::mock

#endif  // TRANSFORMER_INTERFACE_MOCK_HPP_
