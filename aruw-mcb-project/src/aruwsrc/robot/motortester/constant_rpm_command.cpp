#include "constant_rpm_command.hpp"

namespace aruwsrc::motortester
{

ConstantRpmCommand::ConstantRpmCommand(MotorSubsystem* subsystem, float rpm, float pulleyRatio)
    : motorSubsystem(subsystem),
      rpm(rpm),
      pulleyRatio(pulleyRatio)
{
    this->addSubsystemRequirement(subsystem);
}

void ConstantRpmCommand::execute() { motorSubsystem->setDesiredRPM(rpm / pulleyRatio); }

void ConstantRpmCommand::end(bool) { motorSubsystem->stop(); }

}  // namespace aruwsrc::motortester
