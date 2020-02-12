#include "turret_manual_command.hpp"
#include "src/aruwlib/communication/remote.hpp"
#include "turret_subsystem.hpp"

namespace aruwsrc
{

namespace control
{
TurretManualCommand::TurretManualCommand(TurretSubsystem &subsystem) :
    turretSubsystem(subsystem),
    manualYawPid(YAW_P, YAW_I, YAW_D, YAW_MAX_ERROR_SUM, YAW_MAX_OUTPUT),
    manualPitchPid(PITCH_P, PITCH_I, PITCH_D, PITCH_MAX_ERROR_SUM, PITCH_MAX_OUTPUT)
{
    addSubsystemRequirement(&subsystem);
}

float TurretManualCommand::getPitchOutput() {
    return manualPitchPid.getValue();
}

float TurretManualCommand::getYawOutput() {
    return manualYawPid.getValue();
}

void TurretManualCommand::execute() {
    updateTurretPosition();
}

void TurretManualCommand::pitchToVelocity(float velocity) {
    pitchVelocityTarget = velocity;
}

void TurretManualCommand::yawToVelocity(float velocity) {
    yawVelocityTarget = velocity;
}

void TurretManualCommand::pitchIncrementVelocity(float velocity) {
    pitchVelocityTarget += velocity;
}

void TurretManualCommand::yawIncrementVelocity(float velocity) {
    yawVelocityTarget += velocity;
}

void TurretManualCommand::updateTurretPosition() {
    pitchToVelocity(aruwlib::Remote::getChannel(aruwlib::Remote::Channel::RIGHT_VERTICAL) * remoteControlScaler);
    yawToVelocity(aruwlib::Remote::getChannel(aruwlib::Remote::Channel::RIGHT_HORIZONTAL) * remoteControlScaler);
    manualPitchPid.update(pitchVelocityTarget - turretSubsystem.getPitchVelocity());
    manualYawPid.update(yawVelocityTarget - turretSubsystem.getYawVelocity());

    turretSubsystem.setPitchMotorOutput(manualPitchPid.getValue());
    turretSubsystem.setYawMotorOutput(manualYawPid.getValue());
}

}  // control

}  // aruwsrc