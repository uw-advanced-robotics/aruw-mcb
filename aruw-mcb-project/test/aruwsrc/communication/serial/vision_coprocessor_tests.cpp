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
#include "tap/drivers.hpp"
#include "tap/mock/odometry_2d_interface_mock.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/mock/transformer_interface_mock.hpp"
#include "aruwsrc/mock/turret_orientation_interface_mock.hpp"
#include "gtest/gtest.h"

using aruwsrc::serial::VisionCoprocessor;
using tap::communication::serial::DJISerial;
using tap::communication::serial::RefSerialData;
using namespace tap::arch;
using namespace tap::algorithms;
using namespace testing;
using namespace tap::arch::clock;
using namespace aruwsrc::control::turret;

struct TestPositionData
{
    VisionCoprocessor::FireRate firerate;  //.< Firerate of sentry (low 0 - 3 high)

    float xPos;  ///< x position of the target (in m).
    float yPos;  ///< y position of the target (in m).
    float zPos;  ///< z position of the target (in m).

    float xVel;  ///< x velocity of the target (in m/s).
    float yVel;  ///< y velocity of the target (in m/s).
    float zVel;  ///< z velocity of the target (in m/s).

    float xAcc;  ///< x acceleration of the target (in m/s^2).
    float yAcc;  ///< y acceleration of the target (in m/s^2).
    float zAcc;  ///< z acceleration of the target (in m/s^2).

} modm_packed;

struct TestTimingData
{
    uint32_t duration;       ///< duration during which the plate is at the target point
    uint32_t pulseInterval;  ///< time between plate centers transiting the target point
    uint32_t offset;         ///< estimated microseconds beyond "timestamp" at which our
                             ///< next shot should ideally hit
} modm_packed;

struct TestTurretAimDataMessage
{
    uint8_t flags;
    uint32_t timestamp;  ///< timestamp in microseconds
    struct TestPositionData pva;
    struct TestTimingData timing;
} modm_packed;

static void initAndRunAutoAimRxTest(
    std::array<TestTurretAimDataMessage, NUM_TURRETS> expectedAimData)
{
    tap::Drivers drivers;
    VisionCoprocessor serial(&drivers);
    DJISerial::ReceivedSerialMessage message;
    message.header.headByte = 0xA5;
    message.messageType = 2;

    int currIndex = 0;
    for (size_t i = 0; i < expectedAimData.size(); i++)
    {
        int dataLength = VisionCoprocessor::messageWidths::FLAGS_BYTES +
                         VisionCoprocessor::messageWidths::TIMESTAMP_BYTES;
        for (int j = 0; j < VisionCoprocessor::NUM_TAGS; j++)
        {
            if (expectedAimData[i].flags & (1 << j))
            {
                dataLength += VisionCoprocessor::LEN_FIELDS[j];
            }
        }

        message.header.dataLength += dataLength;
        memcpy(message.data + currIndex, &expectedAimData[i], dataLength);
        currIndex += dataLength;
    }

    serial.messageReceiveCallback(message);

    for (size_t i = 0; i < expectedAimData.size(); i++)
    {
        const VisionCoprocessor::TurretAimData &callbackData = serial.getLastAimData(i);
        EXPECT_EQ(expectedAimData[i].pva.xPos, callbackData.pva.xPos);
        EXPECT_EQ(expectedAimData[i].pva.yPos, callbackData.pva.yPos);
        EXPECT_EQ(expectedAimData[i].pva.zPos, callbackData.pva.zPos);
        EXPECT_EQ(expectedAimData[i].pva.xVel, callbackData.pva.xVel);
        EXPECT_EQ(expectedAimData[i].pva.yVel, callbackData.pva.yVel);
        EXPECT_EQ(expectedAimData[i].pva.zVel, callbackData.pva.zVel);
        EXPECT_EQ(expectedAimData[i].pva.xAcc, callbackData.pva.xAcc);
        EXPECT_EQ(expectedAimData[i].pva.yAcc, callbackData.pva.yAcc);
        EXPECT_EQ(expectedAimData[i].pva.zAcc, callbackData.pva.zAcc);
        EXPECT_EQ(expectedAimData[i].flags & 0x1, callbackData.pva.updated);
        EXPECT_EQ(expectedAimData[i].timestamp, callbackData.timestamp);
        EXPECT_EQ(expectedAimData[i].pva.firerate, callbackData.pva.firerate);
    }
}

TEST(VisionCoprocessor, messageReceiveCallback_auto_aim_message_zeros)
{
    std::array<TestTurretAimDataMessage, NUM_TURRETS> aimData = {};
    initAndRunAutoAimRxTest(aimData);
}

TEST(VisionCoprocessor, messageReceiveCallback_auto_aim_message_has_target)
{
    std::array<TestTurretAimDataMessage, NUM_TURRETS> aimData = {};
    aimData[0].flags = 0x1;
    initAndRunAutoAimRxTest(aimData);
}

TEST(VisionCoprocessor, messageReceiveCallback_auto_aim_messages_positive)
{
    std::array<TestTurretAimDataMessage, NUM_TURRETS> aimData = {TestTurretAimDataMessage{
        .flags = 0x1,
        .timestamp = 1234,
        .pva =
            {.firerate = VisionCoprocessor::FireRate::ZERO,
             .xPos = 1,
             .yPos = 2,
             .zPos = 3,
             .xVel = 4,
             .yVel = 5,
             .zVel = 6,
             .xAcc = 7,
             .yAcc = 8,
             .zAcc = 9},
        .timing = {.duration = 0, .pulseInterval = 0, .offset = 0}}};
    initAndRunAutoAimRxTest(aimData);
}

TEST(VisionCoprocessor, messageReceiveCallback_auto_aim_messages_negative)
{
    std::array<TestTurretAimDataMessage, NUM_TURRETS> aimData = {TestTurretAimDataMessage{
        .flags = 0x1,
        .timestamp = 1234,
        .pva =
            {.firerate = VisionCoprocessor::FireRate::ZERO,
             .xPos = -1,
             .yPos = -2,
             .zPos = -3,
             .xVel = -4,
             .yVel = -5,
             .zVel = -6,
             .xAcc = -7,
             .yAcc = -8,
             .zAcc = -9},
        .timing = {.duration = 0, .pulseInterval = 0, .offset = 0}}};
    initAndRunAutoAimRxTest(aimData);
}

TEST(VisionCoprocessor, messageReceiveCallback_auto_aim_messages_decimal)
{
    std::array<TestTurretAimDataMessage, NUM_TURRETS> aimData = {TestTurretAimDataMessage{
        .flags = 0x1,
        .timestamp = 1234,
        .pva =
            {.firerate = VisionCoprocessor::FireRate::ZERO,
             .xPos = -0.45,
             .yPos = -0.35,
             .zPos = -0.25,
             .xVel = -0.15,
             .yVel = -0.05,
             .zVel = 0.05,
             .xAcc = 0.15,
             .yAcc = 0.25,
             .zAcc = 0.35},
        .timing = {.duration = 0, .pulseInterval = 0, .offset = 0}}};
    initAndRunAutoAimRxTest(aimData);
}

TEST(VisionCoprocessor, messageReceiveCallback_auto_aim_messages_large)
{
    std::array<TestTurretAimDataMessage, NUM_TURRETS> aimData = {TestTurretAimDataMessage{
        .flags = 0x1,
        .timestamp = 1234,
        .pva =
            {
                .firerate = VisionCoprocessor::FireRate::ZERO,
                .xPos = 123456789.0f,
                .yPos = 123456789.0f,
                .zPos = 123456789.0f,
                .xVel = 123456789.0f,
                .yVel = 123456789.0f,
                .zVel = 123456789.0f,
                .xAcc = 123456789.0f,
                .yAcc = 123456789.0f,
                .zAcc = 123456789.0f,
            },
        .timing = {
            .duration = 0,
            .pulseInterval = 0,
            .offset = 0,
        }}};

    initAndRunAutoAimRxTest(aimData);
}

TEST(VisionCoprocessor, messageReceiveCallback_multiple_turrets_correct)
{
    std::array<TestTurretAimDataMessage, NUM_TURRETS> aimData = {TestTurretAimDataMessage{
        .flags = 0x1,
        .timestamp = 1234,
        .pva =
            {.firerate = VisionCoprocessor::FireRate::ZERO,
             .xPos = -10,
             .yPos = -0.32,
             .zPos = 234.523,
             .xVel = 12.2,
             .yVel = -90,
             .zVel = 0,
             .xAcc = 76,
             .yAcc = 42,
             .zAcc = -14.2},
        .timing = {.duration = 0, .pulseInterval = 0, .offset = 0}}};

    // if there are > 1 turret, fill in aim data
    for (size_t i = 1; i < aimData.size(); i++)
    {
        aimData[i] = aimData[i - 1];
        aimData[i].pva.firerate =
            (VisionCoprocessor::FireRate)((uint8_t)aimData[i].pva.firerate + 1);
        aimData[i].pva.xPos++;
        aimData[i].pva.yPos++;
        aimData[i].pva.zPos++;
        aimData[i].pva.xVel++;
        aimData[i].pva.yVel++;
        aimData[i].pva.zVel++;
        aimData[i].pva.xAcc++;
        aimData[i].pva.yAcc++;
        aimData[i].pva.zAcc++;
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

TEST(VisionCoprocessor, sendOdometryData_valid_turret_chassis_odom)
{
    ClockStub clock;

    tap::Drivers drivers;
    VisionCoprocessor serial(&drivers);
    testing::NiceMock<aruwsrc::mock::TransformerInterface> transformerInterface;

    serial.attachTransformer(&transformerInterface);
    VisionCoprocessor::OdometryData odometryData;

    tap::algorithms::transforms::Transform turretPose(0.0, 0.0, 0.0, M_PI_2, M_PI_4, M_PI);

    odometryData.timestamp = 132'000;
    odometryData.chassisOdometry.xPos = 10;
    odometryData.chassisOdometry.yPos = -100;
    odometryData.chassisOdometry.zPos = 0;
    odometryData.chassisOdometry.pitch = 0;
    odometryData.chassisOdometry.yaw = 0;
    odometryData.chassisOdometry.roll = 0;

    odometryData.turretOdometry[0].roll = turretPose.getRoll();
    odometryData.turretOdometry[0].pitch = turretPose.getPitch();
    odometryData.turretOdometry[0].yaw = turretPose.getYaw();

    clock.time = odometryData.timestamp / 1000;

    for (size_t i = 1; i < MODM_ARRAY_SIZE(odometryData.turretOdometry); i++)
    {
        odometryData.turretOdometry[i] = odometryData.turretOdometry[i - 1];
        odometryData.turretOdometry[i].pitch++;
        odometryData.turretOdometry[i].yaw++;
        odometryData.turretOdometry[i].roll++;
    }

    tap::algorithms::transforms::Transform chassisPose(
        odometryData.chassisOdometry.xPos,
        odometryData.chassisOdometry.yPos,
        odometryData.chassisOdometry.zPos,
        odometryData.chassisOdometry.roll,
        odometryData.chassisOdometry.pitch,
        odometryData.chassisOdometry.yaw);

    ON_CALL(transformerInterface, getWorldToChassis).WillByDefault(ReturnRef(chassisPose));

    std::vector<tap::algorithms::transforms::Transform> turretTransforms;

    // for now, all turrets have same transform
    // tap::algorithms::transforms::Transform turretPose(
    //     0.0,
    //     0.0,
    //     0.0,
    //     odometryData.turretOdometry[0].roll,
    //     odometryData.turretOdometry[0].pitch,
    //     odometryData.turretOdometry[0].yaw);

    ON_CALL(transformerInterface, getWorldToTurret).WillByDefault(ReturnRef(turretPose));

    static constexpr int HEADER_LEN = 7;
    static constexpr int DATA_LEN = sizeof(VisionCoprocessor::OdometryData);
    static constexpr int CRC16_LEN = 2;
    static constexpr int MSG_LEN = HEADER_LEN + DATA_LEN + CRC16_LEN;

    EXPECT_CALL(drivers.uart, write(_, _, MSG_LEN))
        .WillOnce([&](tap::communication::serial::Uart::UartPort,
                      const uint8_t *data,
                      std::size_t length) {
            DJISerial::SerialMessage<DATA_LEN> msg;
            memcpy(reinterpret_cast<uint8_t *>(&msg), data, MSG_LEN);

            checkHeaderAndTail<DATA_LEN>(msg);
            EXPECT_EQ(1, msg.messageType);

            float cx, cy, cz, roll, pitch, yaw;
            uint32_t timestamp;

            // chassis odometry
            convertFromLittleEndian(&timestamp, msg.data);
            convertFromLittleEndian(&cx, msg.data + 4);
            convertFromLittleEndian(&cy, msg.data + 8);
            convertFromLittleEndian(&cz, msg.data + 12);
            convertFromLittleEndian(&roll, msg.data + 16);
            convertFromLittleEndian(&pitch, msg.data + 20);
            convertFromLittleEndian(&yaw, msg.data + 24);

            EXPECT_EQ(odometryData.timestamp, timestamp);
            EXPECT_EQ(odometryData.chassisOdometry.xPos, cx);
            EXPECT_EQ(odometryData.chassisOdometry.yPos, cy);
            EXPECT_EQ(odometryData.chassisOdometry.zPos, cz);
            EXPECT_EQ(odometryData.chassisOdometry.roll, roll);
            EXPECT_EQ(odometryData.chassisOdometry.pitch, pitch);
            EXPECT_EQ(odometryData.chassisOdometry.yaw, yaw);

            uint8_t numTurrets =
                msg.data[sizeof(uint32_t) + sizeof(VisionCoprocessor::ChassisOdometryData)];
            EXPECT_EQ(NUM_TURRETS, numTurrets);

            // turret odometry
            const uint32_t startIndex = sizeof(timestamp) +
                                        sizeof(VisionCoprocessor::ChassisOdometryData) +
                                        sizeof(numTurrets);
            std::array<std::tuple<float, float, float>, NUM_TURRETS> turretOdom;
            for (size_t i = 0; i < turretOdom.size(); i++)
            {
                auto &odom = turretOdom[i];

                convertFromLittleEndian(
                    &std::get<0>(odom),
                    msg.data + startIndex + i * sizeof(VisionCoprocessor::TurretOdometryData) +
                        0 * sizeof(float));
                convertFromLittleEndian(
                    &std::get<1>(odom),
                    msg.data + startIndex + i * sizeof(VisionCoprocessor::TurretOdometryData) +
                        1 * sizeof(float));
                convertFromLittleEndian(
                    &std::get<2>(odom),
                    msg.data + startIndex + i * sizeof(VisionCoprocessor::TurretOdometryData) +
                        2 * sizeof(float));

                EXPECT_NEAR(turretPose.getRoll(), std::get<0>(odom), 0.01);
                EXPECT_NEAR(turretPose.getPitch(), std::get<1>(odom), 0.01);
                EXPECT_NEAR(turretPose.getYaw(), std::get<2>(odom), 0.01);
            }

            return length;
        });

    serial.sendOdometryData();
}

TEST(VisionCoprocessor, sendRobotTypeData_timer_not_expired_nothing_sent)
{
    ClockStub clock;

    tap::Drivers drivers;
    VisionCoprocessor serial(&drivers);

    EXPECT_CALL(drivers.uart, write(_, _, _)).Times(0);

    serial.sendRobotTypeData();
}

TEST(VisionCoprocessor, sendRobotTypeData_timer_expired_robot_type_sent)
{
    ClockStub clock;

    tap::Drivers drivers;
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

    tap::Drivers drivers;
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

    tap::Drivers drivers;
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

    tap::Drivers drivers;
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
            EXPECT_EQ(10'000'000, *reinterpret_cast<uint32_t *>(msg.data));

            return length;
        });

    VisionCoprocessor::handleTimeSyncRequest();

    // the time we send should be the the time when `handleTimeSyncRequest` was sent
    clock.time += 100;

    serial.sendTimeSyncMessage();
}
