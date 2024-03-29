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

#ifndef CONTROL_OPERATOR_INTERFACE_MOCK_HPP_
#define CONTROL_OPERATOR_INTERFACE_MOCK_HPP_

#include <gmock/gmock.h>

#include "tap/control/command.hpp"
#include "tap/control/subsystem.hpp"

#include "aruwsrc/robot/control_operator_interface.hpp"

namespace aruwsrc
{
namespace mock
{
class ControlOperatorInterfaceMock : public aruwsrc::control::ControlOperatorInterface
{
public:
    ControlOperatorInterfaceMock(tap::Drivers *drivers);
    virtual ~ControlOperatorInterfaceMock();

    MOCK_METHOD(float, getChassisXInput, (), (override));
    MOCK_METHOD(float, getChassisYInput, (), (override));
    MOCK_METHOD(float, getChassisRInput, (), (override));
    MOCK_METHOD(float, getTurretYawInput, (uint8_t), (override));
    MOCK_METHOD(float, getTurretPitchInput, (uint8_t), (override));
    MOCK_METHOD(float, getSentrySpeedInput, (), (override));
};  // class ControlOperatorInterfaceMock
}  // namespace mock
}  // namespace aruwsrc

#endif  // CONTROL_OPERATOR_INTERFACE_MOCK_HPP_
