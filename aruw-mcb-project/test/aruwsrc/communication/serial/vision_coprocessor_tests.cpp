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

// RX tests

static void initAndRunAutoAimRxTest(
    float xPosDesired,
    float yPosDesired,
    float zPosDesired,
    float xVelDesired,
    float yVelDesired,
    float zVelDesired,
    float xAccDesired,
    float yAccDesired,
    float zAccDesired,
    bool hasTarget)
{
    aruwsrc::Drivers drivers;
    VisionCoprocessor serial(&drivers);
    DJISerial::ReceivedSerialMessage message;
    message.header.headByte = 0xA5;
    message.messageType = 2;
    message.header.dataLength = 10 * sizeof(float) + sizeof(uint8_t);
    aruwsrc::serial::VisionCoprocessor::TurretAimData testData;
    testData.xPos = xPosDesired;
    testData.yPos = yPosDesired;
    testData.zPos = zPosDesired;
    testData.xVel = xVelDesired;
    testData.yVel = yVelDesired;
    testData.zVel = zVelDesired;
    testData.xAcc = xAccDesired;
    testData.yAcc = yAccDesired;
    testData.zAcc = zAccDesired;
    testData.hasTarget = hasTarget;
    testData.timestamp = 1234;
    memcpy(&message.data, &testData, sizeof(testData));

    serial.messageReceiveCallback(message);

    const VisionCoprocessor::TurretAimData &callbackData = serial.getLastAimData();
    EXPECT_EQ(hasTarget, callbackData.hasTarget);
    EXPECT_EQ(xPosDesired, callbackData.xPos);
    EXPECT_EQ(yPosDesired, callbackData.yPos);
    EXPECT_EQ(zPosDesired, callbackData.zPos);
    EXPECT_EQ(xVelDesired, callbackData.xVel);
    EXPECT_EQ(yVelDesired, callbackData.yVel);
    EXPECT_EQ(zVelDesired, callbackData.zVel);
    EXPECT_EQ(xAccDesired, callbackData.xAcc);
    EXPECT_EQ(yAccDesired, callbackData.yAcc);
    EXPECT_EQ(zAccDesired, callbackData.zAcc);
    EXPECT_EQ(1234, callbackData.timestamp);
}

TEST(VisionCoprocessor, messageReceiveCallback_auto_aim_message_zeros)
{
    initAndRunAutoAimRxTest(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false);
}

TEST(VisionCoprocessor, messageReceiveCallback_auto_aim_message_has_target)
{
    initAndRunAutoAimRxTest(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, true);
}

TEST(VisionCoprocessor, messageReceiveCallback_auto_aim_messages_positive)
{
    initAndRunAutoAimRxTest(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, false);
}

TEST(VisionCoprocessor, messageReceiveCallback_auto_aim_messages_negative)
{
    initAndRunAutoAimRxTest(-1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f, -7.0f, -8.0f, -9.0f, false);
}

TEST(VisionCoprocessor, messageReceiveCallback_auto_aim_messages_decimal)
{
    initAndRunAutoAimRxTest(
        -0.45f,
        -0.35f,
        -0.25f,
        -0.15f,
        -0.05f,
        0.05f,
        0.15f,
        0.25f,
        0.35f,
        false);
}

TEST(VisionCoprocessor, messageReceiveCallback_auto_aim_messages_large)
{
    initAndRunAutoAimRxTest(
        123456789.0f,
        123456789.0f,
        123456789.0f,
        123456789.0f,
        123456789.0f,
        123456789.0f,
        123456789.0f,
        123456789.0f,
        123456789.0f,
        false);
}

template <uint32_t DATA_LEN>
static void checkHeaderAndTail(const DJISerial::SerialMessage<DATA_LEN> &msg)
{
    EXPECT_EQ(0xa5, msg.header.headByte);
    EXPECT_EQ(DATA_LEN, msg.header.dataLength);
    EXPECT_EQ(calculateCRC8(reinterpret_cast<const uint8_t *>(&msg.header), 4), msg.header.CRC8);
    EXPECT_EQ(calculateCRC16(reinterpret_cast<const uint8_t *>(&msg), sizeof(msg) - 2), msg.CRC16);
}

TEST(VisionCoprocessor, sendOdometryData_nullptr_odomInterface)
{
    ClockStub clock;

    aruwsrc::Drivers drivers;
    VisionCoprocessor serial(&drivers);

    static constexpr int HEADER_LEN = 7;
    static constexpr int DATA_LEN = 24;
    static constexpr int CRC16_LEN = 2;
    static constexpr int MSG_LEN = HEADER_LEN + DATA_LEN + CRC16_LEN;

    ON_CALL(drivers.turretMCBCanComm, getIMUDataTimestamp).WillByDefault(Return(2000));

    EXPECT_CALL(drivers.uart, write(_, _, MSG_LEN))
        .WillOnce([&](tap::communication::serial::Uart::UartPort,
                      const uint8_t *data,
                      std::size_t length) {
            DJISerial::SerialMessage<DATA_LEN> msg;
            memcpy(reinterpret_cast<uint8_t *>(&msg), data, MSG_LEN);

            checkHeaderAndTail<DATA_LEN>(msg);
            EXPECT_EQ(1, msg.messageType);

            float cx, cy, cz, pitch, yaw;
            uint32_t turretImuTime;

            convertFromLittleEndian(&cx, msg.data);
            convertFromLittleEndian(&cy, msg.data + 4);
            convertFromLittleEndian(&cz, msg.data + 8);
            convertFromLittleEndian(&pitch, msg.data + 12);
            convertFromLittleEndian(&yaw, msg.data + 16);
            convertFromLittleEndian(&turretImuTime, msg.data + 20);

            EXPECT_EQ(0, cx);
            EXPECT_EQ(0, cy);
            EXPECT_EQ(0, cz);
            EXPECT_EQ(0, pitch);
            EXPECT_EQ(0, yaw);
            EXPECT_EQ(2000, turretImuTime);

            return length;
        });

    serial.sendOdometryData();
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

TEST(VisionCoprocesor, time_sync_message_sent_after_time_sync_req_received)
{
    ClockStub clock;
    clock.time = 10'000;

    aruwsrc::Drivers drivers;
    VisionCoprocessor serial(&drivers);

    static constexpr int HEADER_LEN = 7;
    static constexpr int DATA_LEN = 4;
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
            EXPECT_EQ(msg.messageType, 10);
            EXPECT_EQ(getTimeMicroseconds(), *reinterpret_cast<uint32_t *>(msg.data));

            return length;
        });

    DJISerial::ReceivedSerialMessage syncRequestMessage;
    syncRequestMessage.header.dataLength = 1;
    syncRequestMessage.messageType = 3;

    serial.messageReceiveCallback(syncRequestMessage);
}
