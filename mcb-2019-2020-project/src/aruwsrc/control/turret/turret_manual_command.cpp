#include "turret_manual_command.hpp"
#include "turret_subsystem.hpp"
#include "src/aruwlib/communication/remote.hpp"

using namespace aruwlib;

namespace aruwsrc
{

namespace control
{
TurretManualCommand::TurretManualCommand(TurretSubsystem *subsystem) :
    turretSubsystem(subsystem),
    manualYawPid(YAW_P, YAW_I, YAW_D, YAW_MAX_ERROR_SUM, YAW_MAX_OUTPUT),
    manualPitchPid(PITCH_P, PITCH_I, PITCH_D, PITCH_MAX_ERROR_SUM, PITCH_MAX_OUTPUT)
{
    addSubsystemRequirement(subsystem);
}

bool TurretManualCommand::isFinished() const
{
    return false;
}

void TurretManualCommand::end(bool) {}

void TurretManualCommand::execute()
{
    updateTurretVelocity();
}

void TurretManualCommand::updateTurretVelocity()
{
    pitchVelocityTarget = getRemoteYMovement() +
                          getMouseXMovement();
    yawVelocityTarget = getRemoteXMovement() +
                        getMouseYMovement();

    manualPitchPid.update(pitchVelocityTarget - turretSubsystem->getPitchVelocity());
    manualYawPid.update(yawVelocityTarget - turretSubsystem->getYawVelocity());

    turretSubsystem->setPitchMotorOutput(manualPitchPid.getValue());
    turretSubsystem->setYawMotorOutput(manualYawPid.getValue());
}

float TurretManualCommand::getRemoteXMovement() const
{
    return Remote::getChannel(Remote::Channel::RIGHT_HORIZONTAL) * REMOTE_INPUT_SCALAR;
}

float TurretManualCommand::getRemoteYMovement() const
{
    return Remote::getChannel(Remote::Channel::RIGHT_VERTICAL) * REMOTE_INPUT_SCALAR;
}

float TurretManualCommand::getMouseXMovement() const
{
    return Remote::getMouseX() * KEYBOARD_INPUT_SCALAR;
}

float TurretManualCommand::getMouseYMovement() const
{
    return Remote::getMouseY() * KEYBOARD_INPUT_SCALAR;
}

}  // namespace control

}  // namespace aruwsrc
