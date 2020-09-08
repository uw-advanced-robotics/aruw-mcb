#ifndef UART_MOCK_HPP_
#define UART_MOCK_HPP_

#include <aruwlib/communication/serial/uart.hpp>
#include <gmock/gmock.h>

class UartMock : public aruwlib::serial::Uart
{
public:
    MOCK_METHOD(bool, read, (aruwlib::serial::Uart::UartPort port, uint8_t *data), (override));
    MOCK_METHOD(
        std::size_t,
        read,
        (aruwlib::serial::Uart::UartPort port, uint8_t *data, std::size_t length),
        (override));
    MOCK_METHOD(
        std::size_t,
        discardReceiveBuffer,
        (aruwlib::serial::Uart::UartPort port),
        (override));
    MOCK_METHOD(bool, write, (aruwlib::serial::Uart::UartPort port, uint8_t data), (override));
    MOCK_METHOD(
        std::size_t,
        write,
        (aruwlib::serial::Uart::UartPort port, const uint8_t *data, std::size_t length),
        (override));
    MOCK_METHOD(bool, isWriteFinished, (aruwlib::serial::Uart::UartPort port), (const override));
};  // class UartMock

#endif  // UART_MOCK_HPP_
