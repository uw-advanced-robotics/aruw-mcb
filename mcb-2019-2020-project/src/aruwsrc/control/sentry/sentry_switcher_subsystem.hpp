#ifndef __SENTRY_SWITCHER_SUBSYSTEM_HPP__
#define __SENTRY_SWITCHER_SUBSYSTEM_HPP__

#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/motor/servo.hpp"

namespace aruwsrc
{

namespace sentry
{

class SentrySwticherSubsystem : public aruwlib::control::Subsystem
{
 public:
    SentrySwticherSubsystem();

    void refresh();

    void useLowerBarrel(bool userLower);

 private:
    static constexpr aruwlib::gpio::Pwm::Pin SWITCHER_SERVO_PIN = aruwlib::gpio::Pwm::Pin::W;

    static constexpr float SWITCHER_SERVO_MIN_PWM = 0.13f;

    static constexpr float SWITCHER_SERVO_MAX_PWM = 0.22f;

    aruwlib::motor::Servo switcherMotor;
};

}  // namespace sentry

}  // aruwsrc

#endif
