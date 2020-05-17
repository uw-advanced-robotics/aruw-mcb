#include <aruwlib/architecture/clock.hpp>
#include "init_pwm_friction_wheel_command.hpp"

namespace aruwsrc
{

namespace drone
{

void InitPWMFrictionWheelCommand::initialize() {
        frictionWheels->setRawFrictionWheelOutput(frictionWheels->MIN_PWM_DUTY);
        zeroThrottleStartTime = aruwlib::arch::clock::getTimeMilliseconds();
}

void InitPWMFrictionWheelCommand::execute() {
    frictionWheels->setRawFrictionWheelOutput(frictionWheels->MIN_PWM_DUTY);
}

bool InitPWMFrictionWheelCommand::isFinished() const {
    return aruwlib::arch::clock::getTimeMilliseconds() - zeroThrottleStartTime > ZERO_THROTTLE_TIME_MS;
}

void InitPWMFrictionWheelCommand::end(bool interrupted) {
       frictionWheels->stopFrictionWheel();
       frictionWheels->initialized = !interrupted;
}

}  // namespace drone
}  // namespace aruwsrc
