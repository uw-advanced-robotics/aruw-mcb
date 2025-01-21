#include "dart_pullback_command.hpp"

#include "dart_launcher_subsystem.hpp"

namespace aruwsrc::robot::dart
{
int ct = 0;
DartPullbackCommand::DartPullbackCommand(DartLauncherSubsystem &dartLauncher)
    : dartLauncher(dartLauncher)
{
    addSubsystemRequirement(&dartLauncher);
}

void DartPullbackCommand::initialize()
{
    ct = -1;
    dartLauncher.moveMotor(power);
}

void DartPullbackCommand::execute()
{
    dartLauncher.moveMotor(power);
    ct += 2;
}

void DartPullbackCommand::end(bool isInterrupted) { dartLauncher.moveMotor(0); }

bool DartPullbackCommand::isFinished() const { return dartLauncher.isBeamBroken(); }

}  // namespace aruwsrc::robot::dart