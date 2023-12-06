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

#include <gtest/gtest.h>

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/communication/serial/inter_robot_signal_transmitter.hpp"
#include "aruwsrc/communication/serial/sentry_strategy_message_types.hpp"

using namespace testing;
using namespace aruwsrc::communication::serial;

enum class mock_message_types : uint8_t
{
    FOO = 0,
    BAR,
    BAZ
};

class InterRobotSignalTransmitterTests : public Test
{
protected:
    InterRobotSignalTransmitterTests()
        : transmitter(
              drivers,
              {tap::communication::serial::RefSerialData::RobotId::BLUE_SENTINEL},
              999)
    {
    }
    void SetUp() override
    {
        ON_CALL(transmitter.refSerialTransmitter, sendRobotToRobotMsg)
            .WillByDefault(Return(modm::ResumableResult<void>(modm::rf::Stop)));
        ON_CALL(drivers.refSerial, getRobotIdBasedOnCurrentRobotTeam)
            .WillByDefault(
                Return(tap::communication::serial::RefSerialData::RobotId::BLUE_SENTINEL));
        ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));
    }

    tap::Drivers drivers;
    InterRobotSignalMessageTransmitter<mock_message_types, 3> transmitter;
    tap::communication::serial::RefSerialData::Rx::RobotData robotData;
};

TEST_F(InterRobotSignalTransmitterTests, send_doesnt_send_when_no_messages_queued)
{
    EXPECT_CALL(transmitter.refSerialTransmitter, sendRobotToRobotMsg).Times(0);
    transmitter.sendQueued();
}

TEST_F(InterRobotSignalTransmitterTests, send_sends_BAR_when_message_queued)
{
    EXPECT_CALL(transmitter.refSerialTransmitter, sendRobotToRobotMsg)
        .Times(1)
        .WillOnce(
            [](tap::communication::serial::RefSerialData::Tx::RobotToRobotMessage* robotToRobotMsg,
               uint16_t,
               tap::communication::serial::RefSerialData::RobotId,
               uint16_t) {
                EXPECT_EQ(
                    static_cast<uint8_t>(mock_message_types::BAR),
                    robotToRobotMsg->dataAndCRC16[0]);
                return modm::ResumableResult<void>(modm::rf::Stop);
            });
    transmitter.queueMessage(mock_message_types::BAR);
    transmitter.sendQueued();
}

TEST_F(InterRobotSignalTransmitterTests, send_sends_FOO_when_message_queued)
{
    EXPECT_CALL(transmitter.refSerialTransmitter, sendRobotToRobotMsg)
        .Times(1)
        .WillOnce(
            [](tap::communication::serial::RefSerialData::Tx::RobotToRobotMessage* robotToRobotMsg,
               uint16_t,
               tap::communication::serial::RefSerialData::RobotId,
               uint16_t) {
                EXPECT_EQ(
                    static_cast<uint8_t>(mock_message_types::FOO),
                    robotToRobotMsg->dataAndCRC16[0]);
                return modm::ResumableResult<void>(modm::rf::Stop);
            });
    transmitter.queueMessage(mock_message_types::FOO);
    transmitter.sendQueued();
}

TEST_F(InterRobotSignalTransmitterTests, send_sends_both_queued_messages)
{
    EXPECT_CALL(transmitter.refSerialTransmitter, sendRobotToRobotMsg).Times(2);
    transmitter.queueMessage(mock_message_types::BAZ);
    transmitter.queueMessage(mock_message_types::FOO);
    transmitter.sendQueued();
}
