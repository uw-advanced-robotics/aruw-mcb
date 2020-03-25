#include "init_friction_wheel_command.hpp"

namespace aruwsrc
{

namespace drone
{

void InitFrictionWheelCommand::initialize() {
    if (!turret->initialized) {
        turret->setRawFrictionWheelOutput(turret->MIN_PWM_DUTY);
        zeroThrottleStartTime = modm::Clock::now().getTime();
    }
}

void InitFrictionWheelCommand::execute() {
    if (!turret->initialized) {
        turret->setRawFrictionWheelOutput(turret->MIN_PWM_DUTY);
    }
}

bool InitFrictionWheelCommand::isFinished() const {
    return modm::Clock::now().getTime() - zeroThrottleStartTime > ZERO_THROTTLE_TIME_MS ||
                    turret->initialized;
}

void InitFrictionWheelCommand::end(bool interrupted) {
    if (!turret->initialized) {
       turret->stopFrictionWheel();
    }
    turret->initialized = !interrupted;
}
}  // namespace drone
}  // namespace aruwsrc
