
#include "drone_turret_subsystem.hpp"

using namespace aruwlib;

using namespace aruwlib::algorithms;


namespace aruwsrc
{

namespace drone
{

void DroneTurretSubsystem::setFrictionWheelOutput(float percentage) {
    if (!initialized){
        stopFrictionWheel();
    }
    else {
        throttleRamp.setTarget(mapValLimited<float>(percentage,
                        0.0f, 1.0f, MIN_PWM_DUTY, MAX_PWM_DUTY));
        lastRampTime = Board::getTimeMicroseconds();
    }
}

void DroneTurretSubsystem::setRawFrictionWheelOutput(float duty) {
    leftFrictionWheel.Write(duty, leftFrictionWheelPin);
    rightFrictionWheel.Write(duty, rightFrictionWheelPin);
    currentFrictionWheelPWMDuty = duty;
}

void DroneTurretSubsystem::stopFrictionWheel() {
    setRawFrictionWheelOutput(MIN_PWM_DUTY);
}

void DroneTurretSubsystem::refresh() {
    throttleRamp.update(RAMP_RATE * (Board::getTimeMicroseconds() - lastRampTime));
    setRawFrictionWheelOutput(throttleRamp.getValue());
    lastRampTime = Board::getTimeMicroseconds();
}

bool DroneTurretSubsystem::isInitialized() {
    return initialized;
}

} // namespace drone
} // namespace aruwsrc
