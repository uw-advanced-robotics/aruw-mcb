
#include "pwm_friction_wheel_subsystem.hpp"

using namespace aruwlib;

using namespace aruwlib::algorithms;
using namespace aruwlib::arch::clock;

namespace aruwsrc
{

namespace drone
{

void PWMFrictionWheelSubsystem::setFrictionWheelOutput(float percentage) {
    if (!initialized) {
        stopFrictionWheel();
    } else {
        throttleRamp.setTarget(mapValLimited(percentage,
                        0.0f, 1.0f, MIN_PWM_DUTY, MAX_PWM_DUTY));
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
    uint32_t t = getTimeMicroseconds();
    throttleRamp.update(RAMP_RATE * (t - lastRampTime));
    setRawFrictionWheelOutput(throttleRamp.getValue());
    lastRampTime = t;

}

bool PWMFrictionWheelSubsystem::isInitialized() {
    return initialized;
}

}  // namespace drone
}  // namespace aruwsrc
