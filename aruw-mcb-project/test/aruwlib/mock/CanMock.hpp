#ifndef MOCK_CAN_HPP_
#define MOCK_CAN_HPP_

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

#include <aruwlib/communication/can/can.hpp>
#include <gmock/gmock.h>

namespace aruwlib
{
namespace mock
{
class CanMock : public aruwlib::can::Can
{
public:
    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(bool, isMessageAvailable, (aruwlib::can::CanBus bus), (const override));
    MOCK_METHOD(
        bool,
        getMessage,
        (aruwlib::can::CanBus bus, modm::can::Message *message),
        (override));
    MOCK_METHOD(bool, isReadyToSend, (aruwlib::can::CanBus bus), (const override));
    MOCK_METHOD(
        bool,
        sendMessage,
        (aruwlib::can::CanBus bus, const modm::can::Message &message),
        (override));
};  // class CanMock
}  // namespace mock
}  // namespace aruwlib

#endif  // MOCK_CAN_HPP_
