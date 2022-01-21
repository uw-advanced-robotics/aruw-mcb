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
    message.length = serial.AIM_DATA_MESSAGE_SIZE;

    memcpy(&message.data[serial.AIM_DATA_MESSAGE_X_POSITION_OFFSET], &xPosDesired, sizeof(uint32_t));
    memcpy(&message.data[serial.AIM_DATA_MESSAGE_Y_POSITION_OFFSET], &yPosDesired, sizeof(uint32_t));
    memcpy(&message.data[serial.AIM_DATA_MESSAGE_Z_POSITION_OFFSET], &zPosDesired, sizeof(uint32_t));
    memcpy(&message.data[serial.AIM_DATA_MESSAGE_X_VELOCITY_OFFSET], &xVelDesired, sizeof(uint32_t));
    memcpy(&message.data[serial.AIM_DATA_MESSAGE_Y_VELOCITY_OFFSET], &yVelDesired, sizeof(uint32_t));
    memcpy(&message.data[serial.AIM_DATA_MESSAGE_Z_VELOCITY_OFFSET], &zVelDesired, sizeof(uint32_t));
    memcpy(&message.data[serial.AIM_DATA_MESSAGE_X_ACCELERATION_OFFSET], &xAccDesired, sizeof(uint32_t));
    memcpy(&message.data[serial.AIM_DATA_MESSAGE_Y_ACCELERATION_OFFSET], &yAccDesired, sizeof(uint32_t));
    memcpy(&message.data[serial.AIM_DATA_MESSAGE_Z_ACCELERATION_OFFSET], &zAccDesired, sizeof(uint32_t));
    message.data[serial.AIM_DATA_MESSAGE_HAS_TARGET_OFFSET] = static_cast<uint8_t>(hasTarget);
    message.messageTimestamp = 1234;

    serial.messageReceiveCallback(message);

    EXPECT_TRUE(serial.lastAimDataValid());
    const VisionCoprocessor::TurretAimData &aimData = serial.getLastAimData();
    EXPECT_FLOAT_EQ(hasTarget, aimData.hasTarget);
    EXPECT_FLOAT_EQ(xPosDesired, aimData.xPos);
    EXPECT_FLOAT_EQ(yPosDesired, aimData.yPos);
    EXPECT_FLOAT_EQ(zPosDesired, aimData.zPos);
    EXPECT_FLOAT_EQ(xVelDesired, aimData.xVel);
    EXPECT_FLOAT_EQ(yVelDesired, aimData.yVel);
    EXPECT_FLOAT_EQ(zVelDesired, aimData.zVel);
    EXPECT_FLOAT_EQ(xAccDesired, aimData.xAcc);
    EXPECT_FLOAT_EQ(yAccDesired, aimData.yAcc);
    EXPECT_FLOAT_EQ(zAccDesired, aimData.zAcc);
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
