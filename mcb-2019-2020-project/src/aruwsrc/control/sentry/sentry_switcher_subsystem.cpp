#include "sentry_switcher_subsystem.hpp"

namespace aruwsrc
{

namespace sentry
{

SentrySwticherSubsystem::SentrySwticherSubsystem() :
        switcherMotor(SWITCHER_SERVO_PIN, SWITCHER_SERVO_MIN_PWM, SWITCHER_SERVO_MAX_PWM, 0.1f)
{}

void SentrySwticherSubsystem::refresh()
{
    switcherMotor.updateSendPwmRamp();
}

void SentrySwticherSubsystem::useLowerBarrel(bool useLower)
{
    switcherMotor.setTargetPwm(useLower ? SWITCHER_SERVO_MIN_PWM : SWITCHER_SERVO_MAX_PWM);
}

}  // namespace sentry

}  // aruwsrc
