#ifndef LEDS_MOCK_HPP_
#define LEDS_MOCK_HPP_

#include <aruwlib/communication/gpio/leds.hpp>
#include <gmock/gmock.h>

class LedsMock : public aruwlib::gpio::Leds
{
public:
    MOCK_METHOD(void, init, (), (override));
    MOCK_METHOD(void, set, (LedPin pin, bool isSet), (override));
};  // class LedsMock

#endif  // LEDS_MOCK_HPP_
