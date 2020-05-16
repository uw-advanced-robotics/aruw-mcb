#include "init_pwm_friction_wheel_command.hpp"

namespace aruwsrc
{

namespace drone
{

void InitPWMFrictionWheelCommand::initialize() {
    if (!turret->initialized) {
        ramp.setTarget(RAMP_TARGET);
        turret->setRawFrictionWheelOutput(turret->MIN_PWM_DUTY);
        zeroThrottleStartTime = modm::Clock::now().getTime();
        lastUpdateTime = zeroThrottleStartTime;
    }
}

void InitPWMFrictionWheelCommand::execute() {
    if (!turret->initialized) {
        uint32_t currentTime = modm::Clock::now().getTime();
        if (!(currentTime - zeroThrottleStartTime < ZERO_THROTTLE_TIME_MS))
        {
            ramp.update((static_cast<float>(currentTime - lastUpdateTime) / RAMP_TIME_MS) *
                    (turret->MAX_PWM_DUTY - turret->MIN_PWM_DUTY));
            turret->setRawFrictionWheelOutput(ramp.getValue());
        }
        lastUpdateTime = currentTime;
    }
}

bool InitPWMFrictionWheelCommand::isFinished() const {
    return ramp.isTargetReached() || turret->initialized;
}

void InitPWMFrictionWheelCommand::end(bool interrupted) {
    if (!turret->initialized) {
       turret->stopFrictionWheel();
    }
    turret->initialized = !interrupted;
}
}  // namespace drone
}  // namespace aruwsrc
