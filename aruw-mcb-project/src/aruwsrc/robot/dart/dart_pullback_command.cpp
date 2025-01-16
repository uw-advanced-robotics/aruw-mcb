#include "dart_pullback_command.hpp"
#include "dart_launcher_subsystem.hpp"

namespace aruwsrc::robot::dart
{
    DartPullbackCommand::DartPullbackCommand (
        DartLauncherSubsystem &dartLauncher
    ): dartLauncher(dartLauncher){}

    void DartPullbackCommand::initialize() {
        dartLauncher.moveMotor(power);
        
    }
    
    void DartPullbackCommand::end(bool isInterrupted) {
        dartLauncher.moveMotor(0);
    }

    bool DartPullbackCommand::isFinished() const {
        return dartLauncher.isBeamBroken();
    }

    
}