#include "manual_pwm_friction_wheel_command.hpp"

namespace aruwsrc
{

namespace drone
{

void PWMFrictionWheelManualCommand::initialize() {
}

void PWMFrictionWheelManualCommand::execute() {
    frictionWheels->setFrictionWheelOutput(aruwlib::Remote::getWheel() / 660.0f);
}

bool PWMFrictionWheelManualCommand::isFinished() const {
    return false;
}

void PWMFrictionWheelManualCommand::end(bool interrupted) {
    if (interrupted) {
        frictionWheels->stopFrictionWheel();
    }
}

}  // namespace drone
}  // namespace aruwsrc
