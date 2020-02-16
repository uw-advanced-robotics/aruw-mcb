#include "turret_init_command.hpp"
#include "turret_subsystem.hpp"
#include "src/aruwlib/control/command.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"

namespace aruwsrc
{

namespace control
{

TurretInitCommand::TurretInitCommand(TurretSubsystem *subsystem) :
    yawTargetEncoder(8160, 0, 8192),
    pitchTargetEncoder(4780, 0, 8192),
    turretSubsystem(subsystem),
    initYawPid(YAW_P, YAW_I, YAW_D, YAW_MAX_ERROR_SUM, YAW_MAX_OUTPUT),
    initPitchPid(PITCH_P, PITCH_I, PITCH_D, PITCH_MAX_ERROR_SUM, PITCH_MAX_OUTPUT)
{
    addSubsystemRequirement(subsystem);
}

void TurretInitCommand::execute() {
    if (turretSubsystem->isTurretOnline()) {
        updateTurretPosition();
    }
}

bool TurretInitCommand::isFinished() const {
    float i = yawTargetEncoder.difference(turretSubsystem->getYawEncoder());
    float j = pitchTargetEncoder.difference(turretSubsystem->getPitchEncoder());
    bool b = turretSubsystem->isTurretOnline();
    return b &&
           abs(i) < 5 &&
           abs(j) < 5;
}

void TurretInitCommand::updateTurretPosition() {

    initPitchPid.update(pitchTargetEncoder.difference(turretSubsystem->getPitchEncoder()));
    initYawPid.update(yawTargetEncoder.difference(turretSubsystem->getYawEncoder()));

    turretSubsystem->setPitchMotorOutput(initPitchPid.getValue());
    turretSubsystem->setYawMotorOutput(initYawPid.getValue());
}

}  // namespace control

}  // namespace aruwsrc
