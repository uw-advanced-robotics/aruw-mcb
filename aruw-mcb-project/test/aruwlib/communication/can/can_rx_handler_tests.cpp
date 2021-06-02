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

#include <aruwlib/Drivers.hpp>
#include <gtest/gtest.h>

#include "aruwlib/mock/CanRxHandlerMock.hpp"
#include "aruwlib/mock/CanRxListenerMock.hpp"

TEST(CanRxHandler, ListenerAttachesSelf)
{
    aruwlib::Drivers drivers;
    aruwlib::mock::CanRxListenerMock listener(&drivers, 0, aruwlib::can::CanBus::CAN_BUS1);

    EXPECT_CALL(drivers.canRxHandler, attachReceiveHandler);
    listener.attachSelfToRxHandler();

    EXPECT_CALL(drivers.canRxHandler, removeReceiveHandler);
}

TEST(CanRxHandler, ListenerAttachesAndDetatchesInArray)
{
    aruwlib::Drivers drivers;
    aruwlib::can::CanRxHandler handler(&drivers);

    for (int i = aruwlib::motor::MOTOR1; i <= aruwlib::motor::MOTOR8; i++)
    {
        aruwlib::mock::CanRxListenerMock listener(&drivers, i, aruwlib::can::CanBus::CAN_BUS1);

        int normalizedId = DJI_MOTOR_NORMALIZED_ID(i);

        handler.attachReceiveHandler(&listener);
        EXPECT_EQ(&listener, handler.getHandlerStore(aruwlib::can::CanBus::CAN_BUS1)[normalizedId]);

        handler.removeReceiveHandler(listener);
        EXPECT_EQ(nullptr, handler.getHandlerStore(aruwlib::can::CanBus::CAN_BUS1)[normalizedId]);
        EXPECT_CALL(drivers.canRxHandler, removeReceiveHandler);
    }
}

TEST(CanRxHandler, MessageIsProcessedByCorrectListener)
{
    aruwlib::Drivers drivers;
    aruwlib::can::CanRxHandler handler(&drivers);
    for (int i = aruwlib::motor::MOTOR1; i <= aruwlib::motor::MOTOR8; i++)
    {
        aruwlib::mock::CanRxListenerMock listener(&drivers, i, aruwlib::can::CanBus::CAN_BUS1);

        handler.attachReceiveHandler(&listener);

        EXPECT_CALL(listener, processMessage);
        const modm::can::Message rxMessage(i);
        handler.processReceivedCanData(
            rxMessage,
            handler.getHandlerStore(aruwlib::can::CanBus::CAN_BUS1),
            8);

        handler.removeReceiveHandler(listener);
        EXPECT_EQ(
            nullptr,
            handler.getHandlerStore(aruwlib::can::CanBus::CAN_BUS1)[DJI_MOTOR_NORMALIZED_ID(i)]);
        EXPECT_CALL(drivers.canRxHandler, removeReceiveHandler);
    }
}

TEST(CanRxHandler, ErrorIsThrownWithOOBMessageID)
{
    aruwlib::Drivers drivers;
    aruwlib::can::CanRxHandler handler(&drivers);
    const modm::can::Message rxMessage(9);

    EXPECT_CALL(drivers.errorController, addToErrorList);
    handler.processReceivedCanData(
        rxMessage,
        handler.getHandlerStore(aruwlib::can::CanBus::CAN_BUS1),
        8);
}
