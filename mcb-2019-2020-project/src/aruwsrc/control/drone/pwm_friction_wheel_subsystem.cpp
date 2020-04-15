
#include "pwm_friction_wheel_subsystem.hpp"

using namespace aruwlib;

using namespace aruwlib::algorithms;


namespace aruwsrc
{

namespace drone
{

void PWMFrictionWheelSubsystem::setFrictionWheelOutput(float percentage) {
    if (!initialized) {
        stopFrictionWheel();
    } else {
        throttleRamp.setTarget(mapValLimited<float>(percentage,
                        0.0f, 1.0f, MIN_PWM_DUTY, MAX_PWM_DUTY));
        lastRampTime = Board::getTimeMicroseconds();
    }
}

void PWMFrictionWheelSubsystem::setRawFrictionWheelOutput(float duty) {
    leftFrictionWheel.Write(duty, leftFrictionWheelPin);
    rightFrictionWheel.Write(duty, rightFrictionWheelPin);
    currentFrictionWheelPWMDuty = duty;
}

void PWMFrictionWheelSubsystem::stopFrictionWheel() {
    setRawFrictionWheelOutput(MIN_PWM_DUTY);
}

void PWMFrictionWheelSubsystem::refresh() {
    throttleRamp.update(RAMP_RATE * (Board::getTimeMicroseconds() - lastRampTime));
    setRawFrictionWheelOutput(throttleRamp.getValue());
    lastRampTime = Board::getTimeMicroseconds();
}

bool PWMFrictionWheelSubsystem::isInitialized() {
    return initialized;
}

}  // namespace drone
}  // namespace aruwsrc
