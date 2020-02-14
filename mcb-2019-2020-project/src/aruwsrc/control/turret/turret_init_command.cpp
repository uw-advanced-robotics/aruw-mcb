#include "turret_init_command.hpp"
#include "turret_subsystem.hpp"
#include "src/aruwlib/control/command.hpp"

namespace aruwsrc
{

namespace control
{

TurretInitCommand::TurretInitCommand(TurretSubsystem *subsystem) :
    turretSubsystem(subsystem),
    initYawPid(YAW_P, YAW_I, YAW_D, YAW_MAX_ERROR_SUM, YAW_MAX_OUTPUT),
    initPitchPid(PITCH_P, PITCH_I, PITCH_D, PITCH_MAX_ERROR_SUM, PITCH_MAX_OUTPUT)
{
    addSubsystemRequirement(subsystem);
}

void TurretInitCommand::execute() {
    updateTurretPosition();
}

bool TurretInitCommand::isFinished() const{
    return initYawPid.getErrorSum() == 0 && initPitchPid.getErrorSum() == 0;
}

void TurretInitCommand::updateTurretPosition() {
    initPitchPid.update(TURRET_START_ANGLE - turretSubsystem->getPitchAngleFromCenter());
    initYawPid.update(TURRET_START_ANGLE - turretSubsystem->getYawAngleFromCenter());

    turretSubsystem->setPitchMotorOutput(initPitchPid.getValue());
    turretSubsystem->setYawMotorOutput(initYawPid.getValue());
}

}  // namespace control

}  // namespace aruwsrc
