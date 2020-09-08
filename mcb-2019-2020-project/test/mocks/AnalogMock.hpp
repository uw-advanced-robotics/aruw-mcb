#ifndef ANALOG_MOCK_HPP_
#define ANALOG_MOCK_HPP_

#include <aruwlib/communication/gpio/analog.hpp>
#include <gmock/gmock.h>

class AnalogMock : public aruwlib::gpio::Analog
{
public:
    MOCK_METHOD(void, init, (), (override));
    MOCK_METHOD(uint16_t, read, (Analog::Pin pin), (const override));
};  // class AnalogMock

#endif  // ANALOG_MOCK_HPP_
