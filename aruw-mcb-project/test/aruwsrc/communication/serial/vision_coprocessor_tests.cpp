/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include <iostream>

#include "tap/algorithms/crc.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/architecture/endianness_wrappers.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/drivers.hpp"
#include "gtest/gtest.h"

using aruwsrc::serial::VisionCoprocessor;
using tap::communication::serial::DJISerial;
using tap::communication::serial::RefSerialData;
using namespace tap::arch;
using namespace tap::algorithms;
using namespace testing;
using namespace tap::arch::clock;
using namespace aruwsrc::control::turret;

static void initAndRunAutoAimRxTest(
    std::array<VisionCoprocessor::TurretAimData, NUM_TURRETS> expectedAimData)
{
    aruwsrc::Drivers drivers;
    VisionCoprocessor serial(&drivers);
    DJISerial::ReceivedSerialMessage message;
    message.header.headByte = 0xA5;
    message.messageType = 2;
    message.header.dataLength = 10 * sizeof(float) + sizeof(uint8_t);

    for (const auto &aimData : expectedAimData)
    {
        memcpy(&message.data, &aimData, sizeof(aimData));
    }

    serial.messageReceiveCallback(message);

    for (size_t i = 0; i < expectedAimData.size(); i++)
    {
        const VisionCoprocessor::TurretAimData &callbackData = serial.getLastAimData(i);
        EXPECT_EQ(expectedAimData[i].hasTarget, callbackData.hasTarget);
        EXPECT_EQ(expectedAimData[i].xPos, callbackData.xPos);
        EXPECT_EQ(expectedAimData[i].yPos, callbackData.yPos);
        EXPECT_EQ(expectedAimData[i].zPos, callbackData.zPos);
        EXPECT_EQ(expectedAimData[i].xVel, callbackData.xVel);
        EXPECT_EQ(expectedAimData[i].yVel, callbackData.yVel);
        EXPECT_EQ(expectedAimData[i].zVel, callbackData.zVel);
        EXPECT_EQ(expectedAimData[i].xAcc, callbackData.xAcc);
        EXPECT_EQ(expectedAimData[i].yAcc, callbackData.yAcc);
        EXPECT_EQ(expectedAimData[i].zAcc, callbackData.zAcc);
        EXPECT_EQ(expectedAimData[i].timestamp, callbackData.timestamp);
    }
}

TEST(VisionCoprocessor, messageReceiveCallback_auto_aim_message_zeros)
{
    std::array<VisionCoprocessor::TurretAimData, NUM_TURRETS> aimData = {};
    initAndRunAutoAimRxTest(aimData);
}

TEST(VisionCoprocessor, messageReceiveCallback_auto_aim_message_has_target)
{
    std::array<VisionCoprocessor::TurretAimData, NUM_TURRETS> aimData = {};
    aimData[0].hasTarget = true;
    initAndRunAutoAimRxTest(aimData);
}

TEST(VisionCoprocessor, messageReceiveCallback_auto_aim_messages_positive)
{
    std::array<VisionCoprocessor::TurretAimData, NUM_TURRETS> aimData = {
        VisionCoprocessor::TurretAimData{
            .xPos = 1,
            .yPos = 2,
            .zPos = 3,
            .xVel = 4,
            .yVel = 5,
            .zVel = 6,
            .xAcc = 7,
            .yAcc = 8,
            .zAcc = 9,
            .hasTarget = false,
            .timestamp = 1234,
        }};
    initAndRunAutoAimRxTest(aimData);
}

TEST(VisionCoprocessor, messageReceiveCallback_auto_aim_messages_negative)
{
    std::array<VisionCoprocessor::TurretAimData, NUM_TURRETS> aimData = {
        VisionCoprocessor::TurretAimData{
            .xPos = -1,
            .yPos = -2,
            .zPos = -3,
            .xVel = -4,
            .yVel = -5,
            .zVel = -6,
            .xAcc = -7,
            .yAcc = -8,
            .zAcc = -9,
            .hasTarget = false,
            .timestamp = 1234,
        }};
    initAndRunAutoAimRxTest(aimData);
}

TEST(VisionCoprocessor, messageReceiveCallback_auto_aim_messages_decimal)
{
    std::array<VisionCoprocessor::TurretAimData, NUM_TURRETS> aimData = {
        VisionCoprocessor::TurretAimData{
            .xPos = -0.45,
            .yPos = -0.35,
            .zPos = -0.25,
            .xVel = -0.15,
            .yVel = -0.05,
            .zVel = 0.05,
            .xAcc = 0.15,
            .yAcc = 0.25,
            .zAcc = 0.35,
            .hasTarget = false,
            .timestamp = 1234,
        }};
    initAndRunAutoAimRxTest(aimData);
}

TEST(VisionCoprocessor, messageReceiveCallback_auto_aim_messages_large)
{
    std::array<VisionCoprocessor::TurretAimData, NUM_TURRETS> aimData = {
        VisionCoprocessor::TurretAimData{
            .xPos = 123456789.0f,
            .yPos = 123456789.0f,
            .zPos = 123456789.0f,
            .xVel = 123456789.0f,
            .yVel = 123456789.0f,
            .zVel = 123456789.0f,
            .xAcc = 123456789.0f,
            .yAcc = 123456789.0f,
            .zAcc = 123456789.0f,
            .hasTarget = false,
            .timestamp = 1234,
        }};

    initAndRunAutoAimRxTest(aimData);
}

TEST(VisionCoprocessor, messageReceiveCallback_multiple_turrets_correct)
{
    std::array<VisionCoprocessor::TurretAimData, NUM_TURRETS> aimData = {
        VisionCoprocessor::TurretAimData{
            .xPos = -10,
            .yPos = -0.32,
            .zPos = 234.523,
            .xVel = 12.2,
            .yVel = -90,
            .zVel = 0,
            .xAcc = 76,
            .yAcc = 42,
            .zAcc = -14.2,
            .hasTarget = true,
            .timestamp = 1234,
        }};

    // if there are > 1 turret, fill in aim data
    for (size_t i = 1; i < aimData.size(); i++)
    {
        aimData[i] = aimData[i - 1];
        aimData[i].xPos++;
        aimData[i].yPos++;
        aimData[i].zPos++;
        aimData[i].xVel++;
        aimData[i].yVel++;
        aimData[i].zVel++;
        aimData[i].xAcc++;
        aimData[i].yAcc++;
        aimData[i].zAcc++;
        aimData[i].timestamp++;
    }

    initAndRunAutoAimRxTest(aimData);
}

template <uint32_t DATA_LEN>
static void checkHeaderAndTail(const DJISerial::SerialMessage<DATA_LEN> &msg)
{
    EXPECT_EQ(0xa5, msg.header.headByte);
    EXPECT_EQ(DATA_LEN, msg.header.dataLength);
    EXPECT_EQ(calculateCRC8(reinterpret_cast<const uint8_t *>(&msg.header), 4), msg.header.CRC8);
    EXPECT_EQ(calculateCRC16(reinterpret_cast<const uint8_t *>(&msg), sizeof(msg) - 2), msg.CRC16);
}

TEST(VisionCoprocessor, sendRobotTypeData_timer_not_expired_nothing_sent)
{
    ClockStub clock;

    aruwsrc::Drivers drivers;
    VisionCoprocessor serial(&drivers);

    EXPECT_CALL(drivers.uart, write(_, _, _)).Times(0);

    serial.sendRobotTypeData();
}

TEST(VisionCoprocessor, sendRobotTypeData_timer_expired_robot_type_sent)
{
    ClockStub clock;

    aruwsrc::Drivers drivers;
    VisionCoprocessor serial(&drivers);

    static constexpr int HEADER_LEN = 7;
    static constexpr int DATA_LEN = 1;
    static constexpr int CRC16_LEN = 2;
    static constexpr int MSG_LEN = HEADER_LEN + DATA_LEN + CRC16_LEN;

    RefSerialData::Rx::RobotData robotData;
    ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));

    robotData.robotId = RefSerialData::RobotId::BLUE_SOLDIER_2;

    EXPECT_CALL(drivers.uart, write(_, _, MSG_LEN))
        .Times(1)
        .WillOnce([&](tap::communication::serial::Uart::UartPort,
                      const uint8_t *data,
                      std::size_t length) {
            DJISerial::SerialMessage<DATA_LEN> msg;
            memcpy(reinterpret_cast<uint8_t *>(&msg), data, MSG_LEN);

            checkHeaderAndTail<1>(msg);
            EXPECT_EQ(6, msg.messageType);

            uint8_t robotId;

            convertFromLittleEndian(&robotId, msg.data);

            EXPECT_EQ(static_cast<uint8_t>(robotData.robotId), robotId);

            return length;
        });

    clock.time = 10'000;

    serial.sendRobotTypeData();
}

TEST(VisionCoprocessor, sendShutdownMessage_sends_blank_msg_with_correct_id)
{
    ClockStub clock;

    aruwsrc::Drivers drivers;
    VisionCoprocessor serial(&drivers);

    static constexpr int HEADER_LEN = 7;
    static constexpr int DATA_LEN = 1;
    static constexpr int CRC16_LEN = 2;
    static constexpr int MSG_LEN = HEADER_LEN + DATA_LEN + CRC16_LEN;

    RefSerialData::Rx::RobotData robotData;
    ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));

    robotData.robotId = RefSerialData::RobotId::BLUE_SOLDIER_2;

    EXPECT_CALL(drivers.uart, write(_, _, MSG_LEN))
        .Times(1)
        .WillOnce([&](tap::communication::serial::Uart::UartPort,
                      const uint8_t *data,
                      std::size_t length) {
            DJISerial::SerialMessage<DATA_LEN> msg;
            memcpy(reinterpret_cast<uint8_t *>(&msg), data, MSG_LEN);

            checkHeaderAndTail<1>(msg);
            EXPECT_EQ(9, msg.messageType);

            return length;
        });

    serial.sendShutdownMessage();
}

TEST(VisionCoprocessor, sendRebootMessage_sends_blank_msg_with_correct_id)
{
    ClockStub clock;

    aruwsrc::Drivers drivers;
    VisionCoprocessor serial(&drivers);

    static constexpr int HEADER_LEN = 7;
    static constexpr int DATA_LEN = 1;
    static constexpr int CRC16_LEN = 2;
    static constexpr int MSG_LEN = HEADER_LEN + DATA_LEN + CRC16_LEN;

    RefSerialData::Rx::RobotData robotData;
    ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));

    robotData.robotId = RefSerialData::RobotId::BLUE_SOLDIER_2;

    EXPECT_CALL(drivers.uart, write(_, _, MSG_LEN))
        .Times(1)
        .WillOnce([&](tap::communication::serial::Uart::UartPort,
                      const uint8_t *data,
                      std::size_t length) {
            DJISerial::SerialMessage<DATA_LEN> msg;
            memcpy(reinterpret_cast<uint8_t *>(&msg), data, MSG_LEN);

            checkHeaderAndTail<1>(msg);
            EXPECT_EQ(8, msg.messageType);

            return length;
        });

    serial.sendRebootMessage();
}

TEST(VisionCoprocessor, time_sync_message_sent_after_time_sync_req_received)
{
    ClockStub clock;
    clock.time = 10'000;

    aruwsrc::Drivers drivers;
    VisionCoprocessor serial(&drivers);

    static constexpr int HEADER_LEN = 7;
    static constexpr int DATA_LEN = 5;
    static constexpr int CRC16_LEN = 2;
    static constexpr int MSG_LEN = HEADER_LEN + DATA_LEN + CRC16_LEN;

    EXPECT_CALL(drivers.uart, write(_, _, MSG_LEN))
        .Times(1)
        .WillOnce([&](tap::communication::serial::Uart::UartPort,
                      const uint8_t *data,
                      std::size_t length) {
            DJISerial::SerialMessage<DATA_LEN> msg;
            memcpy(reinterpret_cast<uint8_t *>(&msg), data, MSG_LEN);

            checkHeaderAndTail<DATA_LEN>(msg);
            EXPECT_EQ(msg.messageType, 11);
            EXPECT_EQ(getTimeMicroseconds(), *reinterpret_cast<uint32_t *>(msg.data));
            EXPECT_EQ(77, msg.data[4]);

            return length;
        });

    DJISerial::ReceivedSerialMessage syncRequestMessage;
    syncRequestMessage.header.dataLength = 1;
    syncRequestMessage.messageType = 10;
    syncRequestMessage.data[0] = 77;

    serial.messageReceiveCallback(syncRequestMessage);
}
