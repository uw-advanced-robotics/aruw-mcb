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

#include "tap/drivers.hpp"

#include "aruwsrc/communication/serial/inter_robot_signal_transmitter.hpp"

using namespace testing;
using namespace aruwsrc::communication::serial;

class SentryRequestTransmitterTest : public Test
{
protected:
    SentryRequestTransmitterTest() : transmitter(&drivers) {}
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
    SentryRequestTransmitter transmitter;
    tap::communication::serial::RefSerialData::Rx::RobotData robotData;
};

TEST_F(SentryRequestTransmitterTest, send_doesnt_send_when_no_messages_queued)
{
    EXPECT_CALL(transmitter.refSerialTransmitter, sendRobotToRobotMsg).Times(0);
    transmitter.sendQueued();
}

TEST_F(SentryRequestTransmitterTest, send_sends_select_new_target_when_message_queued)
{
    EXPECT_CALL(transmitter.refSerialTransmitter, sendRobotToRobotMsg)
        .Times(1)
        .WillOnce(
            [](tap::communication::serial::RefSerialData::Tx::RobotToRobotMessage* robotToRobotMsg,
               uint16_t,
               tap::communication::serial::RefSerialData::RobotId,
               uint16_t) {
                EXPECT_EQ(
                    static_cast<uint8_t>(SentryRequestMessageType::SELECT_NEW_ROBOT),
                    robotToRobotMsg->dataAndCRC16[0]);
                return modm::ResumableResult<void>(modm::rf::Stop);
            });
    transmitter.queueRequest(SentryRequestMessageType::SELECT_NEW_ROBOT);
    transmitter.sendQueued();
}

TEST_F(SentryRequestTransmitterTest, send_sends_target_new_quadrant_when_message_queued)
{
    EXPECT_CALL(transmitter.refSerialTransmitter, sendRobotToRobotMsg)
        .Times(1)
        .WillOnce(
            [](tap::communication::serial::RefSerialData::Tx::RobotToRobotMessage* robotToRobotMsg,
               uint16_t,
               tap::communication::serial::RefSerialData::RobotId,
               uint16_t) {
                EXPECT_EQ(
                    static_cast<uint8_t>(SentryRequestMessageType::TARGET_NEW_QUADRANT),
                    robotToRobotMsg->dataAndCRC16[0]);
                return modm::ResumableResult<void>(modm::rf::Stop);
            });
    transmitter.queueRequest(SentryRequestMessageType::TARGET_NEW_QUADRANT);
    transmitter.sendQueued();
}

TEST_F(SentryRequestTransmitterTest, send_sends_both_queued_messages)
{
    EXPECT_CALL(transmitter.refSerialTransmitter, sendRobotToRobotMsg).Times(2);
    transmitter.queueRequest(SentryRequestMessageType::TARGET_NEW_QUADRANT);
    transmitter.queueRequest(SentryRequestMessageType::SELECT_NEW_ROBOT);
    transmitter.sendQueued();
}
