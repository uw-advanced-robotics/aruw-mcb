#include "control_friction_wheel_command.hpp"

namespace aruwsrc
{

namespace drone
{

void ControlFrictionWheelCommand::initialize() {
}

void ControlFrictionWheelCommand::execute() {
    turret->setFrictionWheelOutput(aruwlib::Remote::getWheel() / 660.0f);
}

bool ControlFrictionWheelCommand::isFinished() const {
    return false;
}

void ControlFrictionWheelCommand::end(bool interrupted) {
    turret->stopFrictionWheel();
}
}
}