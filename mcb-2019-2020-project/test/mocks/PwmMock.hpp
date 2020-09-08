#ifndef PWM_MOCK_HPP_
#define PWM_MOCK_HPP_

#include <aruwlib/communication/gpio/pwm.hpp>
#include <gmock/gmock.h>

class PwmMock : public aruwlib::gpio::Pwm
{
public:
    MOCK_METHOD(void, init, (), (override));
    MOCK_METHOD(void, writeAll, (float duty), (override));
    MOCK_METHOD(void, write, (float duty, aruwlib::gpio::Pwm::Pin pin), (override));
};  // class PwmMock

#endif  //  PWM_MOCK_HPP_
