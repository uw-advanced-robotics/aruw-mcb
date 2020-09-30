#ifndef TERMINAL_SERIAL_MOCK_HPP_
#define TERMINAL_SERIAL_MOCK_HPP_

#include <aruwlib/communication/serial/TerminalSerial.hpp>
#include <gmock/gmock.h>

namespace aruwlib
{
namespace mock
{
class TerminalSerialMock : public communication::serial::TerminalSerial
{
public:
    TerminalSerialMock(Drivers *drivers) : communication::serial::TerminalSerial(drivers) {}
    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, update, (), (override));
    MOCK_METHOD(
        void,
        addHeader,
        (const std::string &header, communication::serial::ITerminalSerialCallback *callback),
        (override));
};  // class TerminalSerialMock
}  // namespace mock
}  // namespace aruwlib

#endif  // TERMINAL_SERIAL_MOCK_HPP_
