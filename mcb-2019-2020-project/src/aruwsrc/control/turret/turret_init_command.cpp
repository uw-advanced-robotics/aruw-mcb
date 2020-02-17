#include "turret_init_command.hpp"
#include "turret_subsystem.hpp"
#include "src/aruwlib/control/command.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"

namespace aruwsrc
{

namespace control
{

TurretInitCommand::TurretInitCommand(TurretSubsystem *subsystem) :
    yawTargetEncoder(90.0f, 0.0f, 360.0f),
    pitchTargetEncoder(90.0f, 0.0f, 360.0f),
    turretSubsystem(subsystem),
    initYawPid(YAW_P, YAW_I, YAW_D, YAW_MAX_ERROR_SUM, YAW_MAX_OUTPUT),
    initPitchPid(PITCH_P, PITCH_I, PITCH_D, PITCH_MAX_ERROR_SUM, PITCH_MAX_OUTPUT)
{
    addSubsystemRequirement(subsystem);
}

void TurretInitCommand::execute() {
    updateTurretPosition();
}

bool TurretInitCommand::isFinished() const {
    return fabs(turretSubsystem->getPitchAngleFromCenter()) < 5.0f
        && fabs(turretSubsystem->getYawAngleFromCenter()) < 5.0f
        && turretSubsystem->isTurretOnline(); 
}

void TurretInitCommand::updateTurretPosition() {
    initPitchPid.update(pitchTargetEncoder.difference(turretSubsystem->getPitchAngle()));
    initYawPid.update(yawTargetEncoder.difference(turretSubsystem->getYawAngle()));
    turretSubsystem->setPitchMotorOutput(initPitchPid.getValue());
    turretSubsystem->setYawMotorOutput(initYawPid.getValue());
}

}  // namespace control

}  // namespace aruwsrc
