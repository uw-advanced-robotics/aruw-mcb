#include "turret_cv_command.hpp"
#include "src/aruwlib/communication/remote.hpp"
#include "turret_subsystem.hpp"

namespace aruwsrc
{

namespace control
{

TurretCVCommand::TurretCVCommand(TurretSubsystem* subsystem) : Command(), 
        turretSubsystem(subsystem)
{
    addSubsystemRequirement(reinterpret_cast<Subsystem*>(subsystem));
    CVPitchPid = new modm::Pid<float>::Parameter(PITCH_P, PITCH_I, PITCH_D, PITCH_MAX_ERROR_SUM, PITCH_MAX_OUTPUT);
    CVYawPid = new modm::Pid<float>::Parameter(YAW_P, YAW_I, YAW_D, YAW_MAX_ERROR_SUM, YAW_MAX_OUTPUT);
}

void TurretCVCommand::execute() {
    updateTurretPosition();
}

void TurretCVCommand::initialize() {
    turretSubsystem->turretStatus = turretSubsystem->CV;
    turretSubsystem->pitchMotorPid.setParameter(*CVPitchPid);
    turretSubsystem->yawMotorPid.setParameter(*CVYawPid);
}

void TurretCVCommand::end(bool interrupted) {
    if (interrupted) {
        // print error message
    }
    turretSubsystem->turretStatus = turretSubsystem->IDLE;
}

bool TurretCVCommand::isFinished() const {
    return turretSubsystem->turretStatus != turretSubsystem->CV;
}
void TurretCVCommand::pitchToEncoder(float encoder) {
    pitchEncoderTarget = encoder;
}

void TurretCVCommand::yawToEncoder(float encoder) {
    yawEncoderTarget = encoder;
}

void TurretCVCommand::pitchIncrementEncoder(float encoder) {
    pitchEncoderTarget += encoder;
}

void TurretCVCommand::yawIncrementEncoder(float encoder) {
    yawEncoderTarget += encoder;
}

void TurretCVCommand::updateTurretPosition() {
    turretSubsystem->pitchMotorPid.update(pitchEncoderTarget - turretSubsystem->pitchMotor.encStore.getEncoderUnwrapped());
    turretSubsystem->yawMotorPid.update(yawEncoderTarget - turretSubsystem->yawMotor.encStore.getEncoderUnwrapped());
}

}  // control

}  // aruwsrc