#ifndef REF_SERIAL_MOCK_HPP_
#define REF_SERIAL_MOCK_HPP_

#include <aruwlib/communication/serial/ref_serial.hpp>
#include <gmock/gmock.h>

class RefSerialMock : public aruwlib::serial::RefSerial
{
public:
    MOCK_METHOD(
        void,
        messageReceiveCallback,
        (const aruwlib::serial::DJISerial::SerialMessage& completeMessage),
        (override));
    MOCK_METHOD(const aruwlib::serial::RefSerial::RobotData&, getRobotData, (), (const override));
    MOCK_METHOD(const aruwlib::serial::RefSerial::GameData&, getGameData, (), (const override));
    MOCK_METHOD(
        void,
        sendDisplayData,
        (const aruwlib::serial::RefSerial::DisplayData& displayData),
        (override));
};  // class RefSerialMock

#endif  // REF_SERIAL_MOCK_HPP_
