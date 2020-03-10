#include "sentinel_switcher_subsystem.hpp"

namespace aruwsrc
{

namespace sentinel
{

SentinelSwticherSubsystem::SentinelSwticherSubsystem() :
        switcherMotor(SWITCHER_SERVO_PIN, SWITCHER_SERVO_MIN_PWM, SWITCHER_SERVO_MAX_PWM, 0.1f)
{
    useLowerBarrel(this->useLower);
}

void SentinelSwticherSubsystem::refresh()
{
    switcherMotor.updateSendPwmRamp();
}

void SentinelSwticherSubsystem::useLowerBarrel(bool useLower)
{
    switcherMotor.setTargetPwm(useLower ? SWITCHER_SERVO_MIN_PWM : SWITCHER_SERVO_MAX_PWM);
    this->useLower = useLower;
}

bool SentinelSwticherSubsystem::isLowerUsed() const
{
    return useLower;
}

}  // namespace sentinel

}  // namespace aruwsrc
