#ifndef XAVIER_SERIAL_MOCK_HPP_
#define XAVIER_SERIAL_MOCK_HPP_

#include <aruwlib/communication/serial/xavier_serial.hpp>
#include <gmock/gmock.h>

class XavierSerialMock : public aruwlib::serial::XavierSerial
{
public:
    MOCK_METHOD(void, initializeCV, (), (override));
    MOCK_METHOD(
        void,
        messageReceiveCallback,
        (const aruwlib::serial::DJISerial::SerialMessage& completeMessage),
        (override));
    MOCK_METHOD(
        void,
        sendMessage,
        (const aruwlib::serial::XavierSerial::IMUData& imuData,
         const aruwlib::serial::XavierSerial::ChassisData& chassisData,
         const aruwlib::serial::XavierSerial::TurretAimData& turretData,
         uint8_t robotId),
        (override));
    MOCK_METHOD(void, beginTargetTracking, (), (override));
    MOCK_METHOD(void, stopTargetTracking, (), (override));
    MOCK_METHOD(
        bool,
        getLastAimData,
        (aruwlib::serial::XavierSerial::TurretAimData * aimData),
        (const override));
};  // class XavierSerialMock

#endif  // XAVIER_SERIAL_MOCK_HPP_
