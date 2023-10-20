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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "tap/drivers.hpp"

#include "aruwsrc/communication/serial/sentry_request_handler.hpp"

using namespace testing;
using namespace aruwsrc::communication::serial;
using namespace tap::communication::serial;

class HandlerMock
{
public:
    MOCK_METHOD0(noStrategyHandlerMock, void());
    MOCK_METHOD0(goToFriendlyBaseHandlerMock, void());
};

class SentryRequestHandlerTest : public Test
{
protected:
    SentryRequestHandlerTest() : handler(&drivers) { i = 0; }

    static void noStrategyHandlerMock() { i++; };
    static void goToFriendlyBaseHandlerMock() { i++; };

    tap::Drivers drivers;
    tap::communication::serial::DJISerial::ReceivedSerialMessage message{};
    SentryRequestHandler handler;
    static int i;
};

int SentryRequestHandlerTest::i = 0;

TEST_F(SentryRequestHandlerTest, callback_called_with_no_message_handlers_added)
{
    message.data[sizeof(RefSerialData::Tx::InteractiveHeader)] = 0;
    handler(message);
}

TEST_F(SentryRequestHandlerTest, callback_called_invalid_message_id_raises_error)
{
    EXPECT_CALL(drivers.errorController, addToErrorList);
    message.data[sizeof(RefSerialData::Tx::InteractiveHeader)] = 200;
    handler(message);
}

TEST_F(SentryRequestHandlerTest, callback_called_message_handlers_invoked)
{
    handler.attachNoStrategyHandler(noStrategyHandlerMock);
    handler.attachGoToFriendlyBaseHandler(goToFriendlyBaseHandlerMock);
    message.data[sizeof(RefSerialData::Tx::InteractiveHeader)] = 0;
    handler(message);
    EXPECT_EQ(1, i);
    message.data[sizeof(RefSerialData::Tx::InteractiveHeader)] = 1;
    handler(message);
    EXPECT_EQ(2, i);
}
