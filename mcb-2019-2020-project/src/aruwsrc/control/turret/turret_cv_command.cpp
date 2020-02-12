#include "turret_cv_command.hpp"
#include "src/aruwlib/communication/remote.hpp"
#include "turret_subsystem.hpp"

namespace aruwsrc
{

namespace control
{

TurretCVCommand::TurretCVCommand(TurretSubsystem &subsystem) : 
    turretSubsystem(subsystem),
    yawTargetAngle(0, 0, 360),
    pitchTargetAngle(0, 0, 360),
    CVYawPid(YAW_P, YAW_I, YAW_D, YAW_MAX_ERROR_SUM, YAW_MAX_OUTPUT),
    CVPitchPid(PITCH_P, PITCH_I, PITCH_D, PITCH_MAX_ERROR_SUM, PITCH_MAX_OUTPUT)
{
    addSubsystemRequirement(&subsystem);
}

void TurretCVCommand::execute() {
    updateTurretPosition();
}

void TurretCVCommand::pitchToEncoder(float encoder) {
    pitchTargetAngle.setValue(encoder);
    pitchTargetAngle.reboundValue();
}

void TurretCVCommand::yawToEncoder(float encoder) {
    yawTargetAngle.setValue(encoder);
    yawTargetAngle.reboundValue();
}

void TurretCVCommand::pitchIncrementEncoder(float encoder) {
    pitchTargetAngle.shiftValue(encoder);
}

void TurretCVCommand::yawIncrementEncoder(float encoder) {
    yawTargetAngle.shiftValue(encoder);
}

void TurretCVCommand::updateTurretPosition() {
    CVPitchPid.update(pitchTargetAngle.difference(turretSubsystem.getPitchAngle()));
    CVYawPid.update(yawTargetAngle.difference(turretSubsystem.getYawAngle()));

    turretSubsystem.setPitchMotorOutput(CVPitchPid.getValue());
    turretSubsystem.setYawMotorOutput(CVYawPid.getValue());
}

}  // control

}  // aruwsrc