/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TURRET_ORIENTATION_INTERFACE_MOCK_HPP_
#define TURRET_ORIENTATION_INTERFACE_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/control/turret/turret_orientation_interface.hpp"

namespace aruwsrc::mock
{
class TurretOrientationInterfaceMock : public control::turret::TurretOrientationInterface
{
public:
    MOCK_METHOD(float, getWorldYaw, (), (const override));
    MOCK_METHOD(float, getWorldPitch, (), (const override));
    MOCK_METHOD(uint32_t, getLastMeasurementTimeMicros, (), (const override));
};
}  // namespace aruwsrc::mock

#endif  // TURRET_ORIENTATION_INTERFACE_MOCK_HPP_
