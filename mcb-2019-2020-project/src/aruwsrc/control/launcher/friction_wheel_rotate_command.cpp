#include "friction_wheel_rotate_command.hpp"

#include <aruwlib/control/subsystem.hpp>

#include "friction_wheel_subsystem.hpp"

using aruwlib::control::Subsystem;

namespace aruwsrc
{
namespace launcher
{
FrictionWheelRotateCommand::FrictionWheelRotateCommand(FrictionWheelSubsystem* subsystem, int speed)
    : frictionWheelSubsystem(subsystem),
      speed(speed)
{
    addSubsystemRequirement(dynamic_cast<Subsystem*>(subsystem));
}

void FrictionWheelRotateCommand::initialize() {}

void FrictionWheelRotateCommand::execute() { frictionWheelSubsystem->setDesiredRpm(speed); }

void FrictionWheelRotateCommand::end(bool) { frictionWheelSubsystem->setDesiredRpm(0); }

bool FrictionWheelRotateCommand::isFinished() const { return false; }
}  // namespace launcher

}  // namespace aruwsrc
