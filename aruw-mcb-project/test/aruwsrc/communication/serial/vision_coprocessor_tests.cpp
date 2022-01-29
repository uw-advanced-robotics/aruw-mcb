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
    // TODO: redo memcpy
    message.length = 10 * sizeof(float) + sizeof(uint8_t);
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
