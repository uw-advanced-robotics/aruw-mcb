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

#ifndef BALSTD_CONTROL_OPERATOR_INTERFACE_MOCK_HPP_
#define BALSTD_CONTROL_OPERATOR_INTERFACE_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/robot/balstd/balstd_control_operator_interface.hpp"

namespace aruwsrc
{
namespace mock
{
class BalstdControlOperatorInterfaceMock : public aruwsrc::balstd::BalstdControlOperatorInterface
{
public:
    BalstdControlOperatorInterfaceMock(tap::Drivers *drivers);
    virtual ~BalstdControlOperatorInterfaceMock();

    MOCK_METHOD(float, getXVel, (), (const override));
    MOCK_METHOD(float, getYawVel, (), (const override));
    MOCK_METHOD(float, getHeightVel, (), (const override));
    MOCK_METHOD(float, getManualLegXForce, (), (const override));
    MOCK_METHOD(float, getManualLegYForce, (), (const override));
    MOCK_METHOD(float, getManualWheelTorque, (), (const override));
};  // class BalstdControlOperatorInterfaceMock
}  // namespace mock
}  // namespace aruwsrc

#endif  // BALSTD_CONTROL_OPERATOR_INTERFACE_MOCK_HPP_
