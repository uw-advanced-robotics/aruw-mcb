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

#include <iostream>

#include <aruwlib/Drivers.hpp>
#include <aruwlib/algorithms/math_user_utils.hpp>
#include <aruwlib/architecture/clock.hpp>
#include <aruwlib/architecture/endianness_wrappers.hpp>
#include <gtest/gtest.h>

#include "aruwsrc/mock/ChassisSubsystemMock.hpp"
#include "aruwsrc/mock/TurretSubsystemMock.hpp"
#include "aruwsrc/mock/XavierSerialMock.hpp"
#include "aruwsrc/serial/xavier_serial.hpp"

using aruwlib::Drivers;
using aruwlib::serial::DJISerial;
using aruwsrc::serial::XavierSerial;
using namespace aruwsrc::mock;
using namespace testing;
using namespace aruwlib::arch;

static constexpr float FIXED_POINT_PRECISION = 0.001f;

// class for accessing internals of XavierSerial class for testing purposes
class XavierSerialTester
{
public:
    XavierSerialTester(XavierSerial *serial) : serial(serial) {}

    XavierSerial::AutoAimRequestState *getCurrAimState()
    {
        return &serial->AutoAimRequest.currAimState;
    }

    bool *getCurrAimRequest() { return &serial->AutoAimRequest.requestType; }

private:
    XavierSerial *serial;
};

// RX tests

static void initAndRunAutoAimRxTest(float pitchDesired, float yawDesired, bool hasTarget)
{
    Drivers drivers;
    XavierSerial serial(&drivers, nullptr, nullptr);
    DJISerial<>::SerialMessage message;
    message.headByte = 0xA5;
    message.type = 0;
    message.length = 9;

    // Store in little endian, todo replace with helper
    convertToLittleEndian(static_cast<int32_t>(pitchDesired / FIXED_POINT_PRECISION), message.data);
    convertToLittleEndian(
        static_cast<int32_t>(yawDesired / FIXED_POINT_PRECISION),
        message.data + sizeof(int32_t));
    message.data[2 * sizeof(int32_t)] = static_cast<uint8_t>(hasTarget);
    message.messageTimestamp = 1234;

    serial.messageReceiveCallback(message);

    EXPECT_TRUE(serial.lastAimDataValid());
    const XavierSerial::TurretAimData &aimData = serial.getLastAimData();
    EXPECT_EQ(hasTarget, aimData.hasTarget);
    EXPECT_NEAR(pitchDesired, aimData.pitch, FIXED_POINT_PRECISION);
    EXPECT_NEAR(yawDesired, aimData.yaw, FIXED_POINT_PRECISION);
    EXPECT_EQ(1234, message.messageTimestamp);
}

TEST(XavierSerial, messageReceiveCallback_turret_aim_message_zeros)
{
    initAndRunAutoAimRxTest(0.0f, 0.0f, false);
}

TEST(XavierSerial, messageReceiveCallback_turret_aim_message_has_target)
{
    initAndRunAutoAimRxTest(0, 0, true);
}

TEST(XavierSerial, messageReceiveCallback_turret_aim_messages_whole_numbers)
{
    // Pitch/yaw values should at least be correct between [0, 360]...test from [-360, 360]
    // since negative numbers should work as well
    for (int i = -360; i < 360; i += 10)
    {
        for (int j = -360; j < 360; j += 10)
        {
            initAndRunAutoAimRxTest(i, j, false);
        }
    }
}

TEST(XavierSerial, messageReceiveCallback_turret_aim_messages_single_decimals)
{
    for (float i = -1; i < 1; i += 0.01)
    {
        for (float j = -1; j < 1; j += 0.01)
        {
            initAndRunAutoAimRxTest(i, j, false);
        }
    }
}

TEST(XavierSerial, messageReceiveCallback_tracking_request_ackn)
{
    Drivers drivers;
    XavierSerial serial(&drivers, nullptr, nullptr);
    XavierSerialTester serialTester(&serial);
    DJISerial<>::SerialMessage message;
    message.headByte = 0xA5;
    message.type = 1;
    message.length = 1;

    serial.messageReceiveCallback(message);

    // Request that started complete is always complete
    EXPECT_EQ(XavierSerial::AUTO_AIM_REQUEST_COMPLETE, *serialTester.getCurrAimState());

    // Send wrong message type, curr aim state won't change
    *serialTester.getCurrAimState() = XavierSerial::AUTO_AIM_REQUEST_SENT;
    message.type = 2;
    serial.messageReceiveCallback(message);
    EXPECT_EQ(XavierSerial::AUTO_AIM_REQUEST_SENT, *serialTester.getCurrAimState());

    // Send correct message type, curr aim state will change to complete
    message.type = 1;
    serial.messageReceiveCallback(message);
    EXPECT_EQ(XavierSerial::AUTO_AIM_REQUEST_COMPLETE, *serialTester.getCurrAimState());

    // Send correct message type, but we are already in the acknowledged, so nothing happens
    serial.messageReceiveCallback(message);
    EXPECT_EQ(XavierSerial::AUTO_AIM_REQUEST_COMPLETE, *serialTester.getCurrAimState());
}

// TX tests

static constexpr int FRAME_HEADER_LENGTH = 7;
static constexpr int CRC_LENGTH = 2;

static void setExpectationsForTxTest(Drivers *drivers, int expectedNumMessagesSent)
{
    EXPECT_CALL(drivers->uart, write(_, _, _)).Times(expectedNumMessagesSent);
    EXPECT_CALL(drivers->uart, isWriteFinished(_))
        .Times(expectedNumMessagesSent)
        .WillRepeatedly(testing::Return(true));
}

TEST(XavierSerial, sendMessage_validate_robot_data)
{
    Drivers drivers;
    TurretSubsystemMock ts(&drivers);
    ChassisSubsystemMock cs(&drivers);
    XavierSerial xs(&drivers, &ts, &cs);

    static constexpr int16_t rfWheelRPMToTest[] = {0, -16000, -12345, 231, 12331, 14098, 16000};
    static constexpr int16_t lfWheelRPMToTest[] = {0, -16000, -14889, -1, 3123, 12000, 16000};
    static constexpr int16_t lbWheelRPMToTest[] = {0, -16000, -534, 123, 12394, 15999, 16000};
    static constexpr int16_t rbWheelRPMToTest[] = {0, -16000, -1, 1, 14, 343, 16000};
    static constexpr float turretYawValsToTest[] =
        {0.0f, -180.0f, -360.0f - 24.0, 36.34f, 120.6f, 180.0f};
    static constexpr float turretPitchValsToTest[] =
        {0.0f, -180.0f, -360.0f, -139.45f, 12.9f, 176.48f, 180.0f};
    static constexpr float axValsToTest[] = {0, -180, -123.45, -2.34, 45.9, 54.65, 120.90, 180};
    static constexpr float ayValsToTest[] = {0, -180, -149.43, -75.9, 34.5, 76.9, 176.32, 180};
    static constexpr float azValsToTest[] = {0, -180, -130.54, -34.32, 56.7, 90.4, 130.4, 180};
    static constexpr float gxValsToTest[] = {0, -180, -158.45, -65.4, 43.9, 130.9, 180};
    static constexpr float gyValsToTest[] = {0, -180, -111.32, -65.2, 12.5, 160.8, 180};
    static constexpr float gzValsToTest[] = {0, -180, -167.9, -1.54, 18.5, 169.8, 180};
    static constexpr float pitValsToTest[] = {0, -180, -178.4, -1.6, 3.13, 142.5, 180};
    static constexpr float rollValsToTest[] = {0, -180, -165.4, -12.3, 4.12, 130, 180};
    static constexpr float yawValsToTest[] = {0, -180, -167.5, -1.2, 13.45, 178.9, 180};
    static constexpr int MESSAGES_TO_SEND = sizeof(yawValsToTest) / sizeof(float);

    setExpectationsForTxTest(&drivers, MESSAGES_TO_SEND);
    EXPECT_CALL(drivers.canRxHandler, removeReceiveHandler).Times(6);
    EXPECT_CALL(drivers.djiMotorTxHandler, removeFromMotorManager).Times(6);

    for (int i = 0; i < MESSAGES_TO_SEND; i++)
    {
        EXPECT_CALL(cs, getLeftBackRpmActual).WillRepeatedly(Return(lbWheelRPMToTest[i]));
        EXPECT_CALL(cs, getLeftFrontRpmActual).WillRepeatedly(Return(lfWheelRPMToTest[i]));
        EXPECT_CALL(cs, getRightBackRpmActual).WillRepeatedly(Return(rbWheelRPMToTest[i]));
        EXPECT_CALL(cs, getRightFrontRpmActual).WillRepeatedly(Return(rfWheelRPMToTest[i]));
        aruwlib::algorithms::ContiguousFloat pit(turretPitchValsToTest[i], -1000, 1000);
        aruwlib::algorithms::ContiguousFloat yaw(turretYawValsToTest[i], -1000, 1000);
        EXPECT_CALL(ts, getPitchAngle).WillRepeatedly(ReturnRef(pit));
        EXPECT_CALL(ts, getYawAngle).WillRepeatedly(ReturnRef(yaw));
        EXPECT_CALL(drivers.mpu6500, getGx).WillRepeatedly(Return(gxValsToTest[i]));
        EXPECT_CALL(drivers.mpu6500, getGy).WillRepeatedly(Return(gyValsToTest[i]));
        EXPECT_CALL(drivers.mpu6500, getGz).WillRepeatedly(Return(gzValsToTest[i]));
        EXPECT_CALL(drivers.mpu6500, getAx).WillRepeatedly(Return(axValsToTest[i]));
        EXPECT_CALL(drivers.mpu6500, getAy).WillRepeatedly(Return(ayValsToTest[i]));
        EXPECT_CALL(drivers.mpu6500, getAz).WillRepeatedly(Return(azValsToTest[i]));
        EXPECT_CALL(drivers.mpu6500, getYaw).WillRepeatedly(Return(yawValsToTest[i]));
        EXPECT_CALL(drivers.mpu6500, getPitch).WillRepeatedly(Return(pitValsToTest[i]));
        EXPECT_CALL(drivers.mpu6500, getRoll).WillRepeatedly(Return(rollValsToTest[i]));

        auto checkExpectations =
            [&](aruwlib::serial::Uart::UartPort, const uint8_t *data, std::size_t length) {
                EXPECT_EQ(
                    FRAME_HEADER_LENGTH + 4 * sizeof(int16_t) + 11 * sizeof(int32_t) + CRC_LENGTH,
                    length);

                data += FRAME_HEADER_LENGTH;

                // Chassis data
                int16_t lf, lb, rf, rb;
                int32_t turretPit, turretYaw, gx, gy, gz, ax, ay, az, yaw, pit, roll;
                convertFromLittleEndian(&lf, data);
                convertFromLittleEndian(&lb, data + sizeof(int16_t));
                convertFromLittleEndian(&rf, data + 2 * sizeof(int16_t));
                convertFromLittleEndian(&rb, data + 3 * sizeof(int16_t));
                convertFromLittleEndian(&turretPit, data + 4 * sizeof(int16_t));
                convertFromLittleEndian(&turretYaw, data + 4 * sizeof(int16_t) + sizeof(int32_t));
                convertFromLittleEndian(&gx, data + 4 * sizeof(int16_t) + 2 * sizeof(int32_t));
                convertFromLittleEndian(&gy, data + 4 * sizeof(int16_t) + 3 * sizeof(int32_t));
                convertFromLittleEndian(&gz, data + 4 * sizeof(int16_t) + 4 * sizeof(int32_t));
                convertFromLittleEndian(&ax, data + 4 * sizeof(int16_t) + 5 * sizeof(int32_t));
                convertFromLittleEndian(&ay, data + 4 * sizeof(int16_t) + 6 * sizeof(int32_t));
                convertFromLittleEndian(&az, data + 4 * sizeof(int16_t) + 7 * sizeof(int32_t));
                convertFromLittleEndian(&yaw, data + 4 * sizeof(int16_t) + 8 * sizeof(int32_t));
                convertFromLittleEndian(&pit, data + 4 * sizeof(int16_t) + 9 * sizeof(int32_t));
                convertFromLittleEndian(&roll, data + 4 * sizeof(int16_t) + 10 * sizeof(int32_t));

                EXPECT_EQ(lfWheelRPMToTest[i], lf);
                EXPECT_EQ(lbWheelRPMToTest[i], lb);
                EXPECT_EQ(rfWheelRPMToTest[i], rf);
                EXPECT_EQ(rbWheelRPMToTest[i], rb);
                EXPECT_NEAR(
                    turretPitchValsToTest[i],
                    static_cast<float>(turretPit) * FIXED_POINT_PRECISION,
                    FIXED_POINT_PRECISION);
                EXPECT_NEAR(
                    turretYawValsToTest[i],
                    static_cast<float>(turretYaw) * FIXED_POINT_PRECISION,
                    FIXED_POINT_PRECISION);
                EXPECT_NEAR(
                    gxValsToTest[i],
                    static_cast<float>(gx) * FIXED_POINT_PRECISION,
                    FIXED_POINT_PRECISION);
                EXPECT_NEAR(
                    gyValsToTest[i],
                    static_cast<float>(gy) * FIXED_POINT_PRECISION,
                    FIXED_POINT_PRECISION);
                EXPECT_NEAR(
                    gzValsToTest[i],
                    static_cast<float>(gz) * FIXED_POINT_PRECISION,
                    FIXED_POINT_PRECISION);
                EXPECT_NEAR(
                    axValsToTest[i],
                    static_cast<float>(ax) * FIXED_POINT_PRECISION,
                    FIXED_POINT_PRECISION);
                EXPECT_NEAR(
                    ayValsToTest[i],
                    static_cast<float>(ay) * FIXED_POINT_PRECISION,
                    FIXED_POINT_PRECISION);
                EXPECT_NEAR(
                    azValsToTest[i],
                    static_cast<float>(az) * FIXED_POINT_PRECISION,
                    FIXED_POINT_PRECISION);
                EXPECT_NEAR(
                    yawValsToTest[i],
                    static_cast<float>(yaw) * FIXED_POINT_PRECISION,
                    FIXED_POINT_PRECISION);
                EXPECT_NEAR(
                    pitValsToTest[i],
                    static_cast<float>(pit) * FIXED_POINT_PRECISION,
                    FIXED_POINT_PRECISION);
                EXPECT_NEAR(
                    rollValsToTest[i],
                    static_cast<float>(roll) * FIXED_POINT_PRECISION,
                    FIXED_POINT_PRECISION);

                return length;
            };

        ON_CALL(drivers.uart, write(_, _, _)).WillByDefault(checkExpectations);

        xs.sendMessage();
    }
}

TEST(XavierSerial, sendMessage_validate_robot_ID)
{
    static constexpr float TIME_BETWEEN_ROBOT_ID_SEND = 5000;
    static constexpr int ROBOT_IDS_TO_CHECK =
        aruwlib::serial::RefSerial::BLUE_SENTINEL - aruwlib::serial::RefSerial::RED_HERO;

    Drivers drivers;
    XavierSerial xs(&drivers, nullptr, nullptr);
    aruwlib::serial::RefSerial::RobotData robotData;

    setExpectationsForTxTest(&drivers, ROBOT_IDS_TO_CHECK);
    EXPECT_CALL(drivers.refSerial, getRobotData)
        .Times(ROBOT_IDS_TO_CHECK)
        .WillRepeatedly(ReturnRef(robotData));
    ON_CALL(drivers.uart, write(_, _, _))
        .WillByDefault(
            [&](aruwlib::serial::Uart::UartPort, const uint8_t *data, std::size_t length) {
                data += FRAME_HEADER_LENGTH;
                EXPECT_EQ(length, FRAME_HEADER_LENGTH + 1 + CRC_LENGTH);
                EXPECT_EQ(robotData.robotId, data[0]);
                return length;
            });

    xs.initializeCV();

    aruwlib::arch::clock::setTime(0);

    for (int i = aruwlib::serial::RefSerial::RED_HERO;
         i <= aruwlib::serial::RefSerial::BLUE_SENTINEL;
         i++)
    {
        aruwlib::arch::clock::setTime(
            aruwlib::arch::clock::getTimeMilliseconds() + TIME_BETWEEN_ROBOT_ID_SEND);
        robotData.robotId = static_cast<aruwlib::serial::RefSerial::RobotId>(i);
        xs.sendMessage();
    }
}

TEST(XavierSerial, beginAutoAim_starts_aim_request)
{
    Drivers drivers;
    XavierSerial xs(&drivers, nullptr, nullptr);
    XavierSerialTester xst(&xs);

    xs.beginAutoAim();
    EXPECT_EQ(*xst.getCurrAimState(), XavierSerial::AUTO_AIM_REQUEST_QUEUED);
    EXPECT_EQ(true, *xst.getCurrAimRequest());

    *xst.getCurrAimState() = XavierSerial::AUTO_AIM_REQUEST_QUEUED;
    *xst.getCurrAimRequest() = false;
    xs.beginAutoAim();
    EXPECT_EQ(*xst.getCurrAimState(), XavierSerial::AUTO_AIM_REQUEST_QUEUED);
    EXPECT_EQ(true, *xst.getCurrAimRequest());

    *xst.getCurrAimState() = XavierSerial::AUTO_AIM_REQUEST_SENT;
    *xst.getCurrAimRequest() = false;
    xs.beginAutoAim();
    EXPECT_EQ(*xst.getCurrAimState(), XavierSerial::AUTO_AIM_REQUEST_QUEUED);
    EXPECT_EQ(true, *xst.getCurrAimRequest());
}

TEST(XavierSerial, stopAutoAim_stops_auto_aim_req)
{
    Drivers drivers;
    XavierSerial xs(&drivers, nullptr, nullptr);
    XavierSerialTester xst(&xs);

    xs.stopAutoAim();
    EXPECT_EQ(*xst.getCurrAimState(), XavierSerial::AUTO_AIM_REQUEST_QUEUED);
    EXPECT_EQ(false, *xst.getCurrAimRequest());

    *xst.getCurrAimState() = XavierSerial::AUTO_AIM_REQUEST_QUEUED;
    *xst.getCurrAimRequest() = true;
    xs.stopAutoAim();
    EXPECT_EQ(*xst.getCurrAimState(), XavierSerial::AUTO_AIM_REQUEST_QUEUED);
    EXPECT_EQ(false, *xst.getCurrAimRequest());

    *xst.getCurrAimState() = XavierSerial::AUTO_AIM_REQUEST_SENT;
    *xst.getCurrAimRequest() = true;
    xs.stopAutoAim();
    EXPECT_EQ(*xst.getCurrAimState(), XavierSerial::AUTO_AIM_REQUEST_QUEUED);
    EXPECT_EQ(false, *xst.getCurrAimRequest());
}

TEST(XavierSerial, sendMessage_validate_begin_target_tracking_request)
{
    Drivers drivers;
    XavierSerial xs(&drivers, nullptr, nullptr);
    XavierSerialTester xst(&xs);
    bool autoAimRequest;

    setExpectationsForTxTest(&drivers, 2);
    ON_CALL(drivers.uart, write(_, _, _))
        .WillByDefault(
            [&](aruwlib::serial::Uart::UartPort, const uint8_t *data, std::size_t length) {
                EXPECT_EQ(FRAME_HEADER_LENGTH + 1 + CRC_LENGTH, length);
                EXPECT_EQ(autoAimRequest, data[FRAME_HEADER_LENGTH]);
                return length;
            });

    aruwlib::arch::clock::setTime(0);
    xs.initializeCV();

    // Queue a message
    xs.beginAutoAim();
    autoAimRequest = true;

    // Send target tracking request
    xs.sendMessage();

    // We shall now be waiting for a reply from the Xavier
    EXPECT_EQ(*xst.getCurrAimState(), XavierSerial::AUTO_AIM_REQUEST_SENT);

    // Send a message from the Xavier
    DJISerial<>::SerialMessage message;
    message.length = 1;
    message.type = 1;
    message.data[0] = 1;
    xs.messageReceiveCallback(message);

    // The request was acknowledged, so the request state should be complete
    EXPECT_EQ(*xst.getCurrAimState(), XavierSerial::AUTO_AIM_REQUEST_COMPLETE);

    // Queue another message to stop auto aiming
    xs.stopAutoAim();
    autoAimRequest = false;

    // Send target tracking request
    xs.sendMessage();

    // We shall now be waiting for a reply from the Xavier
    EXPECT_EQ(*xst.getCurrAimState(), XavierSerial::AUTO_AIM_REQUEST_SENT);

    // Send a message from the Xavier
    message.length = 1;
    message.type = 1;
    message.data[0] = 1;
    xs.messageReceiveCallback(message);

    // The request was acknowledged, so the request state should be complete
    EXPECT_EQ(*xst.getCurrAimState(), XavierSerial::AUTO_AIM_REQUEST_COMPLETE);
}

TEST(XavierSerial, sendMessage_resend_if_msg_not_acknowledged)
{
    static constexpr uint32_t RESEND_AIM_REQUEST_TIMEOUT = 1000;

    Drivers drivers;
    XavierSerial xs(&drivers, nullptr, nullptr);
    XavierSerialTester xst(&xs);
    bool autoAimRequest;

    setExpectationsForTxTest(&drivers, 2);
    ON_CALL(drivers.uart, write(_, _, _))
        .WillByDefault(
            [&](aruwlib::serial::Uart::UartPort, const uint8_t *data, std::size_t length) {
                EXPECT_EQ(FRAME_HEADER_LENGTH + 1 + CRC_LENGTH, length);
                EXPECT_EQ(autoAimRequest, data[FRAME_HEADER_LENGTH]);
                return length;
            });

    aruwlib::arch::clock::setTime(0);
    xs.initializeCV();

    // Queue a message
    xs.beginAutoAim();
    autoAimRequest = true;

    // Send target tracking request
    xs.sendMessage();

    // We shall now be waiting for a reply from the Xavier
    EXPECT_EQ(*xst.getCurrAimState(), XavierSerial::AUTO_AIM_REQUEST_SENT);

    // Set the time s.t. the resend timer times out and resend message
    aruwlib::arch::clock::setTime(RESEND_AIM_REQUEST_TIMEOUT);
    xs.sendMessage();

    EXPECT_EQ(*xst.getCurrAimState(), XavierSerial::AUTO_AIM_REQUEST_SENT);
}
