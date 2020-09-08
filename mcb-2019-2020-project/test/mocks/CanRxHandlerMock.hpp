#ifndef CAN_RX_HANDLER_MOCK_HPP_
#define CAN_RX_HANDLER_MOCK_HPP_

#include <aruwlib/communication/can/can_rx_handler.hpp>
#include <gmock/gmock.h>

class CanRxHandlerMock : public aruwlib::can::CanRxHandler
{
public:
    MOCK_METHOD(
        void,
        attachReceiveHandler,
        (aruwlib::can::CanRxListener* const listener),
        (override));
    MOCK_METHOD(void, pollCanData, (), (override));
    MOCK_METHOD(
        void,
        removeReceiveHandler,
        (const aruwlib::can::CanRxListener& rxListener),
        (override));
};  // class CanRxHandlerMock

#endif  //  CAN_RX_HANDLER_MOCK_HPP_
