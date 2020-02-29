#include "init_friction_wheel_command.hpp"

namespace aruwsrc
{

namespace drone
{

void InitFrictionWheelCommand::initialize() {
    ramp.setTarget(turret->MAX_PWM_DUTY);
    turret->setRawFrictionWheelOutput(turret->MIN_PWM_DUTY);
    zeroThrottleStartTime = modm::Clock::now().getTime();
    lastUpdateTime = zeroThrottleStartTime;
}

void InitFrictionWheelCommand::execute() {
    uint32_t currentTime = modm::Clock::now().getTime();
    if (!(currentTime - zeroThrottleStartTime < ZERO_THROTTLE_TIME_MS))
    {
        ramp.update(((float)(currentTime - lastUpdateTime) / RAMP_TIME_MS) *
                (turret->MAX_PWM_DUTY - turret->MIN_PWM_DUTY));
        turret->setRawFrictionWheelOutput(ramp.getValue());
    }
    lastUpdateTime = currentTime;
}

bool InitFrictionWheelCommand::isFinished() const {
    return ramp.isTargetReached() || turret->initialized;
}

void InitFrictionWheelCommand::end(bool interrupted) {
    turret->stopFrictionWheel();
    turret->initialized = !interrupted;
}
}
}