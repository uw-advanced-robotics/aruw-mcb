#include "control_friction_wheel_command.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
namespace aruwsrc
{

namespace drone
{

void ControlFrictionWheelCommand::initialize() {
}
float frictionwheelspeed = 0.0f;
void ControlFrictionWheelCommand::execute() {
    // frictionwheelspeed = aruwlib::algorithms::limitVal<float>(frictionwheelspeed + aruwlib::Remote::getWheel() / 660.0f / 2000.0f, 0.0f, 1.0f);
    // turret->setFrictionWheelOutput(frictionwheelspeed);
    turret->setFrictionWheelOutput(aruwlib::Remote::getWheel() / 660.0f);
}

bool ControlFrictionWheelCommand::isFinished() const {
    return false;
}

void ControlFrictionWheelCommand::end(bool interrupted) {
    if (interrupted) {
        turret->stopFrictionWheel();
    }
}

}  // namespace drone
}  // namespace aruwsrc
