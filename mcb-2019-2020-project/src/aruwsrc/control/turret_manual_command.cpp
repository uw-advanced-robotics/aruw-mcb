#include "turret_manual_command.hpp"
#include "src/aruwlib/communication/remote.hpp"
#include "src/aruwsrc/control/turret_subsystem.hpp"

namespace aruwsrc
{

namespace control
{

TurretManualCommand::TurretManualCommand(TurretSubsystem* subsystem) : Command(), 
        turretSubsystem(subsystem)
{
    addSubsystemRequirement(reinterpret_cast<Subsystem*>(subsystem));
    manualPitchPid = new modm::Pid<float>::Parameter(PITCH_P, PITCH_I, PITCH_D);
    manualYawPid = new modm::Pid<float>::Parameter(YAW_P, YAW_I, YAW_D);
}

void TurretManualCommand::execute() {
    updateTurretPosition();
}

void TurretManualCommand::initialize() {
    turretSubsystem->setTurretCV;
    turretSubsystem->pitchMotorPid.setParameter(*manualPitchPid);
    turretSubsystem->yawMotorPid.setParameter(*manualYawPid);
}

void TurretManualCommand::end(bool interrupted) {
    if (interrupted) {
        // print error message
    }
    turretSubsystem->setTurretIdle;
}

bool TurretManualCommand::isFinished() const {
    return turretSubsystem->turretStatus != turretSubsystem->CV;
}

void TurretManualCommand::updateTurretPosition() {
    turretSubsystem->incPitchMotorByDegree(aruwlib::Remote::getChannel(aruwlib::Remote::Channel::RIGHT_VERTICAL));
    turretSubsystem->incYawMotorByDegree(aruwlib::Remote::getChannel(aruwlib::Remote::Channel::RIGHT_HORIZONTAL));
}

}  // control

}  // aruwsrc