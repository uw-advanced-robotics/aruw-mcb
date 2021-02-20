#include <iostream>
#include "aruwsrc/mock/XavierSerialMock.hpp"

#include <aruwlib/Drivers.hpp>
#include <aruwlib/algorithms/math_user_utils.hpp>
#include "aruwsrc/serial/xavier_serial.hpp"
#include <gtest/gtest.h>

using aruwlib::Drivers;
using aruwlib::serial::DJISerial;
using aruwsrc::serial::XavierSerial;
using namespace testing;

// class for accessing internals of XavierSerial class for testing purposes
class XavierSerialTester
{
public:
    XavierSerialTester(XavierSerial *serial) : serial(serial) {}

    XavierSerial::AutoAimRequestState *getCurrAimState() { return &serial->AutoAimRequest.currAimState; }
private:
    XavierSerial *serial;
};

static constexpr float FIXED_POINT_PRECISION = 0.001f;

static void initAndRunAutoAimRxTest(float pitchDesired, float yawDesired, bool hasTarget)
{
    Drivers drivers;
    XavierSerial serial(&drivers);
    DJISerial<>::SerialMessage message;
    message.headByte = 0xA5;
    message.type = 0;
    message.length = 9;
    int32_t pitchRounded = static_cast<int32_t>(pitchDesired / FIXED_POINT_PRECISION);
    int32_t yawRounded = static_cast<int32_t>(yawDesired / FIXED_POINT_PRECISION);

    // Store in little endian, todo replace with helper
    message.data[0] = pitchRounded;
    message.data[1] = pitchRounded >> 8;
    message.data[2] = pitchRounded >> 16;
    message.data[3] = pitchRounded >> 24;
    message.data[4] = yawRounded;
    message.data[5] = yawRounded >> 8;
    message.data[6] = yawRounded >> 16;
    message.data[7] = yawRounded >> 24;
    message.data[8] = static_cast<uint8_t>(hasTarget);
    message.messageTimestamp = 1234;

    serial.messageReceiveCallback(message);

    EXPECT_TRUE(serial.lastAimDataValid());
    const XavierSerial::TurretAimData &aimData = serial.getLastAimData();
    EXPECT_EQ(hasTarget, aimData.hasTarget);
    ASSERT_NEAR(pitchDesired, aimData.pitch, FIXED_POINT_PRECISION);
    ASSERT_NEAR(yawDesired, aimData.yaw, FIXED_POINT_PRECISION);
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

TEST(XavierSerial, messageReceiveCallback_tracking_request_ackn) {
    Drivers drivers;
    XavierSerial serial(&drivers);
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

// static constexpr int FRAME_HEADER_LENGTH = 7;
// static constexpr int CRC_16_LENGTH = 2;
// static constexpr int TX_MESSAGE_TYPES = 4;

// /**
//  * Convertss 2 8 bit integers to a 16 bit integer.
//  */
// static int16_t littleEndianInt8ToInt16(const uint8_t *data)
// {
//     return static_cast<int16_t>(
//         static_cast<int16_t>(data[0]) | (static_cast<int16_t>(data[1]) << 8));
// }

// /**
//  * Converts 2 8 bit integers to a 16 bit integer, converts this value to a float,
//  * and divides by 100.
//  *
//  * @param[in] data The array containing the 2 8 bit values, assumed to be at least
//  *      2 bytes long and not nullptr.
//  * @return The converted value.
//  */
// static float convertBitsToFloat(const uint8_t *data)
// {
//     return static_cast<float>(littleEndianInt8ToInt16(data)) / 100.0f;
// }

// static void setExpectationsForTxTest(Drivers &drivers, int expectedNumMessagesSent)
// {
//     EXPECT_CALL(drivers.uart, write(_, _, _)).Times(expectedNumMessagesSent);
//     EXPECT_CALL(drivers.uart, isWriteFinished(_))
//         .Times(expectedNumMessagesSent)
//         .WillRepeatedly(testing::Return(true));
// }

// static void cycleOtherMessageTypes(
//     Drivers &drivers,
//     XavierSerial &serial,
//     int messagesToCycleThrough)
// {
//     // Cycle through other message types
//     ON_CALL(drivers.uart, write(_, _, _))
//         .WillByDefault([](aruwlib::serial::Uart::UartPort, const uint8_t *, std::size_t length) {
//             return length;
//         });
//     for (int j = 0; j < messagesToCycleThrough; j++)
//     {
//         serial.sendMessage(
//             XavierSerial::IMUData(),
//             XavierSerial::ChassisData(),
//             XavierSerial::TurretAimData(),
//             0);
//     }
// }

// static void validateTurretDataTx(
//     const uint8_t *data,
//     std::size_t length,
//     float pitchExpected,
//     float yawExpected)
// {
//     EXPECT_EQ(FRAME_HEADER_LENGTH + 4 + CRC_16_LENGTH, length);

//     float pitch = convertBitsToFloat(data + FRAME_HEADER_LENGTH);
//     float yaw = convertBitsToFloat(data + FRAME_HEADER_LENGTH + 2);

//     ASSERT_NEAR(pitchExpected, pitch, 0.01f);
//     ASSERT_NEAR(yawExpected, yaw, 0.01f);
// }

// TEST(XavierSerial, sendMessageVariableTurretData)
// {
//     Drivers drivers;
//     XavierSerial serial(&drivers);

//     XavierSerial::IMUData id;
//     XavierSerial::ChassisData cd;
//     XavierSerial::TurretAimData tad;

//     static constexpr float yawValsToTest[] = {0.0f, -180.0f, -24.0, 36.34f, 120.6f, 180.0f};
//     static constexpr float pitchValsToTest[] = {0.0f, -180.0f, -139.45f, 12.9f, 176.48f, 180.0f};

//     static constexpr int MESSAGES_TO_SEND = sizeof(yawValsToTest) / sizeof(float);

//     setExpectationsForTxTest(drivers, (TX_MESSAGE_TYPES - 2) * MESSAGES_TO_SEND);

//     for (int i = 0; i < MESSAGES_TO_SEND; i++)
//     {
//         tad.pitch = pitchValsToTest[i];
//         tad.yaw = yawValsToTest[i];

//         ON_CALL(drivers.uart, write(_, _, _))
//             .WillByDefault(
//                 [tad](aruwlib::serial::Uart::UartPort, const uint8_t *data, std::size_t length) {
//                     validateTurretDataTx(data, length, tad.pitch, tad.yaw);
//                     return length;
//                 });
//         serial.sendMessage(id, cd, tad, 0);

//         cycleOtherMessageTypes(drivers, serial, TX_MESSAGE_TYPES - 1);
//     }
// }

// static void validateIMUChassisData(
//     const uint8_t *data,
//     std::size_t length,
//     const XavierSerial::IMUData &imuDataExpected,
//     const XavierSerial::ChassisData &chassisDataExpected)
// {
//     EXPECT_EQ(FRAME_HEADER_LENGTH + 26 + CRC_16_LENGTH, length);

//     data = data + FRAME_HEADER_LENGTH;

//     float ax = convertBitsToFloat(data);
//     float ay = convertBitsToFloat(data + 2);
//     float az = convertBitsToFloat(data + 4);
//     float roll = convertBitsToFloat(data + 6);
//     float pit = convertBitsToFloat(data + 8);
//     float yaw = convertBitsToFloat(data + 10);
//     float wx = convertBitsToFloat(data + 12);
//     float wy = convertBitsToFloat(data + 14);
//     float wz = convertBitsToFloat(data + 16);
//     int16_t rfWheelRPM = littleEndianInt8ToInt16(data + 18);
//     int16_t lfWheelRPM = littleEndianInt8ToInt16(data + 20);
//     int16_t lbWheelRPM = littleEndianInt8ToInt16(data + 22);
//     int16_t rbWheelRPM = littleEndianInt8ToInt16(data + 24);

//     ASSERT_NEAR(imuDataExpected.ax, ax, 0.01f);
//     ASSERT_NEAR(imuDataExpected.ay, ay, 0.01f);
//     ASSERT_NEAR(imuDataExpected.az, az, 0.01f);
//     ASSERT_NEAR(imuDataExpected.wx, wx, 0.01f);
//     ASSERT_NEAR(imuDataExpected.wy, wy, 0.01f);
//     ASSERT_NEAR(imuDataExpected.wz, wz, 0.01f);
//     ASSERT_NEAR(imuDataExpected.pit, pit, 0.01f);
//     ASSERT_NEAR(imuDataExpected.rol, roll, 0.01f);
//     ASSERT_NEAR(imuDataExpected.yaw, yaw, 0.01f);
//     ASSERT_NEAR(chassisDataExpected.rightFrontWheelRPM, rfWheelRPM, 0.01f);
//     ASSERT_NEAR(chassisDataExpected.leftFrontWheelRPM, lfWheelRPM, 0.01f);
//     ASSERT_NEAR(chassisDataExpected.leftBackWheeRPM, lbWheelRPM, 0.01f);
//     ASSERT_NEAR(chassisDataExpected.rightBackWheelRPM, rbWheelRPM, 0.01f);
// }

// TEST(XavierSerial, sendMessageIMUChassis)
// {
//     static constexpr float axValsToTest[] = {0, -180, -123.45, -2.34, 45.9, 54.65, 120.90, 180};
//     static constexpr float ayValsToTest[] = {0, -180, -149.43, -75.9, 34.5, 76.9, 176.32, 180};
//     static constexpr float azValsToTest[] = {0, -180, -130.54, -34.32, 56.7, 90.4, 130.4, 180};
//     static constexpr float wxValsToTest[] = {0, -180, -158.45, -65.4, 43.9, 130.9, 180};
//     static constexpr float wyValsToTest[] = {0, -180, -111.32, -65.2, 12.5, 160.8, 180};
//     static constexpr float wzValsToTest[] = {0, -180, -167.9, -1.54, 18.5, 169.8, 180};
//     static constexpr float pitValsToTest[] = {0, -180, -178.4, -1.6, 3.13, 142.5, 180};
//     static constexpr float rollValsToTest[] = {0, -180, -165.4, -12.3, 4.12, 130, 180};
//     static constexpr float yawValsToTest[] = {0, -180, -167.5, -1.2, 13.45, 178.9, 180};
//     static constexpr int16_t rfWheelRPMToTest[] = {0, -16000, -12345, 231, 12331, 14098, 16000};
//     static constexpr int16_t lfWheelRPMToTest[] = {0, -16000, -14889, -1, 3123, 12000, 16000};
//     static constexpr int16_t lbWheelRPMToTest[] = {0, -16000, -534, 123, 12394, 15999, 16000};
//     static constexpr int16_t rbWheelRPMToTest[] = {0, -16000, -1, 1, 14, 343, 16000};
//     static constexpr int MESSAGES_TO_SEND = sizeof(axValsToTest) / sizeof(float);

//     Drivers drivers;
//     XavierSerial serial(&drivers);

//     XavierSerial::IMUData id;
//     XavierSerial::ChassisData cd;
//     XavierSerial::TurretAimData tad;

//     setExpectationsForTxTest(drivers, (TX_MESSAGE_TYPES - 2) * MESSAGES_TO_SEND + 1);

//     // The imu and chassis data is the second message to be queued, so send a message initially
//     // such that during testing the first message to be send in the cycle is the imu and chassis
//     // data.
//     cycleOtherMessageTypes(drivers, serial, 1);

//     for (int i = 0; i < MESSAGES_TO_SEND; i++)
//     {
//         id.ax = axValsToTest[i];
//         id.ay = ayValsToTest[i];
//         id.az = azValsToTest[i];
//         id.wx = wxValsToTest[i];
//         id.wy = wyValsToTest[i];
//         id.wz = wzValsToTest[i];
//         id.pit = pitValsToTest[i];
//         id.rol = rollValsToTest[i];
//         id.yaw = yawValsToTest[i];
//         cd.leftBackWheeRPM = lbWheelRPMToTest[i];
//         cd.leftFrontWheelRPM = lfWheelRPMToTest[i];
//         cd.rightBackWheelRPM = rbWheelRPMToTest[i];
//         cd.rightFrontWheelRPM = rfWheelRPMToTest[i];

//         ON_CALL(drivers.uart, write(_, _, _))
//             .WillByDefault(
//                 [id, cd](aruwlib::serial::Uart::UartPort, const uint8_t *data, std::size_t length) {
//                     validateIMUChassisData(data, length, id, cd);
//                     return length;
//                 });
//         serial.sendMessage(id, cd, tad, 0);

//         cycleOtherMessageTypes(drivers, serial, TX_MESSAGE_TYPES - 1);
//     }
// }

// static void setOnCallTargetTracking(Drivers &drivers, bool targetTrackingEnabled)
// {
//     ON_CALL(drivers.uart, write(_, _, _))
//         .WillByDefault([targetTrackingEnabled](
//                            aruwlib::serial::Uart::UartPort,
//                            const uint8_t *data,
//                            std::size_t length) {
//             EXPECT_EQ(FRAME_HEADER_LENGTH + 1 + CRC_16_LENGTH, length);
//             EXPECT_EQ(targetTrackingEnabled, static_cast<bool>(data[FRAME_HEADER_LENGTH]));
//             return length;
//         });
// }

// TEST(XavierSerial, beginTargetTrackingEnablesTrackingMessage)
// {
//     static constexpr int MESSAGE_INDEX = 3;

//     Drivers drivers;
//     XavierSerial serial(&drivers);

//     XavierSerial::IMUData id;
//     XavierSerial::ChassisData cd;
//     XavierSerial::TurretAimData tad;

//     setExpectationsForTxTest(drivers, (TX_MESSAGE_TYPES - 2) * 2 + 1 + 2);

//     cycleOtherMessageTypes(drivers, serial, MESSAGE_INDEX);

//     serial.beginTargetTracking();
//     setOnCallTargetTracking(drivers, true);
//     serial.sendMessage(id, cd, tad, 0);
//     cycleOtherMessageTypes(drivers, serial, TX_MESSAGE_TYPES - 1);
//     ON_CALL(drivers.uart, write(_, _, _))
//         .WillByDefault([](aruwlib::serial::Uart::UartPort, const uint8_t *, std::size_t length) {
//             EXPECT_TRUE(false);
//             return length;
//         });
//     serial.sendMessage(id, cd, tad, 0);
//     cycleOtherMessageTypes(drivers, serial, TX_MESSAGE_TYPES - 1);
// }

// TEST(XavierSerial, endTargetTrackingDisablesTrackingMessage)
// {
//     static constexpr int MESSAGE_INDEX = 3;

//     Drivers drivers;
//     XavierSerial serial(&drivers);

//     XavierSerial::IMUData id;
//     XavierSerial::ChassisData cd;
//     XavierSerial::TurretAimData tad;

//     setExpectationsForTxTest(drivers, (TX_MESSAGE_TYPES - 2) * 2 + 1 + 2);

//     cycleOtherMessageTypes(drivers, serial, MESSAGE_INDEX);

//     serial.stopTargetTracking();
//     setOnCallTargetTracking(drivers, false);
//     serial.sendMessage(id, cd, tad, 0);
//     cycleOtherMessageTypes(drivers, serial, TX_MESSAGE_TYPES - 1);
//     ON_CALL(drivers.uart, write(_, _, _))
//         .WillByDefault([](aruwlib::serial::Uart::UartPort, const uint8_t *, std::size_t length) {
//             EXPECT_TRUE(false);
//             return length;
//         });
//     serial.sendMessage(id, cd, tad, 0);
//     cycleOtherMessageTypes(drivers, serial, TX_MESSAGE_TYPES - 1);
// }
