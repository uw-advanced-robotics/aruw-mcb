#include "constant_rpm_command.hpp"

ConstantRpmCommand::ConstantRpmCommand(MotorSubsystem* subsystem, float rpm, float pulleyRatio)
    : motorSubsystem(subsystem),
      rpm(rpm),
      pulleyRatio(pulleyRatio)
{
    this->addSubsystemRequirement(subsystem);
}

void ConstantRpmCommand::execute()
{
    motorSubsystem->setDesiredRPM(rpm / pulleyRatio);
    motorSubsystem->refresh();
}

void ConstantRpmCommand::end(bool) { motorSubsystem->stop(); }