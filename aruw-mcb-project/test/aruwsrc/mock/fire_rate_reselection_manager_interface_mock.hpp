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

#ifndef FIRE_RATE_RESELECTION_MANAGER_INTERFACE_MOCK_HPP_
#define FIRE_RATE_RESELECTION_MANAGER_INTERFACE_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/control/agitator/fire_rate_reselection_manager_interface.hpp"

namespace aruwsrc::mock
{
class FireRateReselectionManagerInterfaceMock
    : public control::agitator::FireRateReselectionManagerInterface
{
public:
    MOCK_METHOD(uint32_t, getFireRatePeriod, (), (override));
    MOCK_METHOD(
        control::agitator::FireRateReadinessState,
        getFireRateReadinessState,
        (),
        (override));
};
}  // namespace aruwsrc::mock

#endif  // FIRE_RATE_RESELECTION_MANAGER_INTERFACE_MOCK_HPP_
