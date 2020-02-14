#include "turret_cv_command.hpp"
#include "src/aruwlib/communication/remote.hpp"
#include "turret_subsystem.hpp"

namespace aruwsrc
{

namespace control
{

TurretCVCommand::TurretCVCommand(TurretSubsystem *subsystem) :
    turretSubsystem(subsystem),
    yawTargetAngle(0, 0, 360),
    pitchTargetAngle(0, 0, 360),
    CVYawPid(YAW_P, YAW_I, YAW_D, YAW_MAX_ERROR_SUM, YAW_MAX_OUTPUT),
    CVPitchPid(PITCH_P, PITCH_I, PITCH_D, PITCH_MAX_ERROR_SUM, PITCH_MAX_OUTPUT)
{
    addSubsystemRequirement(subsystem);
}

void TurretCVCommand::initialize()
{
    // add xavier stuff here
}

void TurretCVCommand::execute() {
    updateTurretPosition();
}

void TurretCVCommand::pitchIncrementEncoder(float encoder) {
    pitchTargetAngle.shiftValue(encoder);
}

void TurretCVCommand::yawIncrementEncoder(float encoder) {
    yawTargetAngle.shiftValue(encoder);
}

void TurretCVCommand::updateTurretPosition() {
    CVPitchPid.update(pitchTargetAngle.difference(turretSubsystem->getPitchAngleFromCenter()));
    CVYawPid.update(yawTargetAngle.difference(turretSubsystem->getYawAngleFromCenter()));

    turretSubsystem->setPitchMotorOutput(CVPitchPid.getValue());
    turretSubsystem->setYawMotorOutput(CVYawPid.getValue());
}

}  // namespace control

}  // namespace aruwsrc
