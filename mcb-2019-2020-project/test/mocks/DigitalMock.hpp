#ifndef DIGITAL_MOCK_HPP_
#define DIGITAL_MOCK_HPP_

#include <aruwlib/communication/gpio/digital.hpp>
#include <gmock/gmock.h>

class DigitalMock : public aruwlib::gpio::Digital
{
public:
    MOCK_METHOD(void, init, (), (override));
    MOCK_METHOD(
        void,
        configureInputPullMode,
        (aruwlib::gpio::Digital::InputPin pin, aruwlib::gpio::Digital::InputPullMode mode),
        (override));
    MOCK_METHOD(void, set, (aruwlib::gpio::Digital::OutputPin pin, bool isSet), (override));
    MOCK_METHOD(bool, read, (aruwlib::gpio::Digital::InputPin pin), (const override));
};  // class DigitalMock

#endif  // DIGITAL_MOCK_HPP_
