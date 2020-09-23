#include "KillAllCommand.hpp"

#include <aruwlib/Drivers.hpp>

namespace aruwlib
{
namespace control
{
KillAllCommand::KillAllCommand(KillAllSubsystem *killAllSubsystem, aruwlib::Drivers *drivers)
    : killAllSubsystem(killAllSubsystem),
      drivers(drivers)
{
    addSubsystemRequirement(killAllSubsystem);
}

void KillAllCommand::initialize() { drivers->commandScheduler.enterKillMode(this); }

void KillAllCommand::execute() {}

void KillAllCommand::end(bool) { drivers->commandScheduler.exitKillMode(); }

bool KillAllCommand::isFinished() const { return false; }
}  // namespace control
}  // namespace aruwlib
