#include "dart_release_command.hpp"
#include "dart_launcher_subsystem.hpp"

namespace aruwsrc::robot::dart
{
    DartReleaseCommand::DartReleaseCommand (
        DartLauncherSubsystem &dartLauncher
    ): dartLauncher(dartLauncher){
        addSubsystemRequirement(&dartLauncher);
    }

    void DartReleaseCommand::initialize() {
        dartLauncher.moveMotor(power);
        
    }

    void DartReleaseCommand::end(bool isInterrupted) {
        dartLauncher.moveMotor(0);
    }

    bool DartReleaseCommand::isFinished() const {
       // return dartLauncher.isLimitSwitched();
       return false;
    }

    
}