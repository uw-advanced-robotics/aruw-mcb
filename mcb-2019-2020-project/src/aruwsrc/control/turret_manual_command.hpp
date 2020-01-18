#ifndef __TURRET_MANUAL_COMMAND_H__
#define __TURRET_MANUAL_COMMAND_H__

#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/command.hpp"
#include "src/aruwsrc/control/turret_subsystem.hpp"

namespace aruwsrc
{

namespace control
{

class TurretManualCommand : public Command {
 public:
    explicit TurretManualCommand(void) {
        addSubsystemRequirement(&TurretSubsystem);
    }

    void initialize() {
        TurretSubsystem.turretStatus = TurretSubsystem.CV;
    }

    void execute() {
        updateTurretPosition();
    }

    void end(bool interrupted) {
        if (interrupted) {
            // print error message
        }
        TurretSubsystem.turretStatus = TurretSubsystem.IDLE;
    }

    bool isFinished() {
        return TurretSubsystem.turretStatus != TurretSubsystem.CV;
    }

 private:
    modm::Pid<float> manualYawPid;
    modm::Pid<float> manualPitchPid;

    TurretSubsystem TurretSubsystem;

    void updateTurretPosition();
};

}  // control

}  // aruwsrc

#endif