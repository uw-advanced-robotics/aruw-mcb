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

#ifndef DJI_MOTOR_TERMINAL_SERIAL_HANDLER_MOCK_HPP_
#define DJI_MOTOR_TERMINAL_SERIAL_HANDLER_MOCK_HPP_

#include <aruwlib/motor/DjiMotorTerminalSerialHandler.hpp>
#include <gmock/gmock.h>

namespace aruwlib
{
namespace mock
{
class DjiMotorTerminalSerialHandlerMock : public motor::DjiMotorTerminalSerialHandler
{
public:
    DjiMotorTerminalSerialHandlerMock(Drivers *drivers)
        : motor::DjiMotorTerminalSerialHandler(drivers)
    {
    }
    MOCK_METHOD(void, init, (), (override));
    MOCK_METHOD(bool, terminalSerialCallback, (char *, modm::IOStream &, bool), (override));
    MOCK_METHOD(void, terminalSerialStreamCallback, (modm::IOStream &), (override));
};
}  // namespace mock
}  // namespace aruwlib

#endif  //  DJI_MOTOR_TERMINAL_SERIAL_HANDLER_MOCK_HPP_
