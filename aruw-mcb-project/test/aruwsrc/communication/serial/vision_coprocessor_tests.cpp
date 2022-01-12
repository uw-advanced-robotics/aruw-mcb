/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/architecture/endianness_wrappers.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/drivers.hpp"
#include "gtest/gtest.h"

using aruwsrc::serial::VisionCoprocessor;
using tap::serial::DJISerial;
using namespace tap::arch;


// class for accessing internals of VisionCoprocessor class for testing purposes
class VisionCoprocessorTester
{
public:
    VisionCoprocessorTester(VisionCoprocessor *serial) : serial(serial) {}

    VisionCoprocessor::AutoAimRequestState *getCurrAimState()
    {
        return &serial->AutoAimRequest.currAimState;
    }

    bool *getCurrAimRequest() { return &serial->AutoAimRequest.autoAimRequest; }

private:
    VisionCoprocessor *serial;
};

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
    DJISerial::SerialMessage message;
    message.headByte = 0xA5;
    message.type = 0;
    message.length = 10 * sizeof(uint32_t) + sizeof(uint8_t);

    convertToLittleEndian(*reinterpret_cast<uint32_t *>(&xPosDesired), message.data);
    convertToLittleEndian(*reinterpret_cast<uint32_t *>(&yPosDesired), message.data);
    convertToLittleEndian(*reinterpret_cast<uint32_t *>(&zPosDesired), message.data);
    convertToLittleEndian(*reinterpret_cast<uint32_t *>(&xVelDesired), message.data);
    convertToLittleEndian(*reinterpret_cast<uint32_t *>(&yVelDesired), message.data);
    convertToLittleEndian(*reinterpret_cast<uint32_t *>(&zVelDesired), message.data);
    convertToLittleEndian(*reinterpret_cast<uint32_t *>(&xAccDesired), message.data);
    convertToLittleEndian(*reinterpret_cast<uint32_t *>(&yAccDesired), message.data);
    convertToLittleEndian(*reinterpret_cast<uint32_t *>(&zAccDesired), message.data);
    message.data[8 * sizeof(uint32_t) + sizeof(uint8_t)] = static_cast<uint8_t>(hasTarget);
    message.messageTimestamp = 1234;

    serial.messageReceiveCallback(message);

    EXPECT_TRUE(serial.lastAimDataValid());
    const VisionCoprocessor::TurretAimData &aimData = serial.getLastAimData();
    EXPECT_EQ(hasTarget, aimData.hasTarget);
    EXPECT_EQ(xPosDesired, aimData.xPos);
    EXPECT_EQ(xPosDesired, aimData.yPos);
    EXPECT_EQ(xPosDesired, aimData.zPos);
    EXPECT_EQ(xPosDesired, aimData.xVel);
    EXPECT_EQ(xPosDesired, aimData.yVel);
    EXPECT_EQ(xPosDesired, aimData.zVel);
    EXPECT_EQ(xPosDesired, aimData.xAcc);
    EXPECT_EQ(xPosDesired, aimData.yAcc);
    EXPECT_EQ(xPosDesired, aimData.zAcc);
    EXPECT_EQ(1234, message.messageTimestamp);
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
    initAndRunAutoAimRxTest(-0.45f, -0.35f, -0.25f, -0.15f, -0.05f, 0.05f, 0.15f, 0.25f, 0.35f, false);
}

TEST(VisionCoprocessor, messageReceiveCallback_auto_aim_messages_large)
{
    initAndRunAutoAimRxTest(
        123456789.0f, 123456789.0f, 123456789.0f,
        123456789.0f, 123456789.0f, 123456789.0f,
        123456789.0f, 123456789.0f, 123456789.0f,
        false);
}

TEST(VisionCoprocessor, messageReceiveCallback_tracking_request_ackn)
{
    aruwsrc::Drivers drivers;
    VisionCoprocessor serial(&drivers);
    VisionCoprocessorTester serialTester(&serial);
    DJISerial::SerialMessage message;
    message.headByte = 0xA5;
    message.type = 0;
    message.length = 5;
    memset(message.data, 0, 5);

    serial.messageReceiveCallback(message);

    // Request that started complete is always complete
    EXPECT_EQ(VisionCoprocessor::AUTO_AIM_REQUEST_COMPLETE, *serialTester.getCurrAimState());

    // Send wrong message type, curr aim state won't change
    *serialTester.getCurrAimState() = VisionCoprocessor::AUTO_AIM_REQUEST_SENT;
    message.type = 2;
    serial.messageReceiveCallback(message);
    EXPECT_EQ(VisionCoprocessor::AUTO_AIM_REQUEST_SENT, *serialTester.getCurrAimState());

    // Send correct message type, curr aim state will change to complete
    message.type = 0;
    serial.messageReceiveCallback(message);
    EXPECT_EQ(VisionCoprocessor::AUTO_AIM_REQUEST_COMPLETE, *serialTester.getCurrAimState());

    // Send correct message type, but we are already in the acknowledged, so nothing happens
    serial.messageReceiveCallback(message);
    EXPECT_EQ(VisionCoprocessor::AUTO_AIM_REQUEST_COMPLETE, *serialTester.getCurrAimState());
}
