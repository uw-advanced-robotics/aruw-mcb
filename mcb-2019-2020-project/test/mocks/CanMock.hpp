#ifndef MOCK_CAN_HPP_
#define MOCK_CAN_HPP_

#include <aruwlib/communication/can/can.hpp>
#include <gmock/gmock.h>

class CanMock : public aruwlib::can::Can
{
public:
    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(bool, isMessageAvailable, (aruwlib::can::CanBus bus), (const override));
    MOCK_METHOD(
        bool,
        getMessage,
        (aruwlib::can::CanBus bus, modm::can::Message *message),
        (override));
    MOCK_METHOD(bool, isReadyToSend, (aruwlib::can::CanBus bus), (const override));
    MOCK_METHOD(
        bool,
        sendMessage,
        (aruwlib::can::CanBus bus, const modm::can::Message &message),
        (override));
};  // class CanMock

#endif  // MOCK_CAN_HPP_
