#include "drone_turret_manual_command.hpp"
#include "drone_turret_subsystem.hpp"
#include <aruwlib/communication/remote.hpp>
#include <aruwlib/control/control_operator_interface.hpp>
#include <aruwlib/algorithms/math_user_utils.hpp>

using namespace aruwlib;

namespace aruwsrc
{

namespace turret
{
DroneTurretManualCommand::DroneTurretManualCommand(DroneTurretSubsystem *subsystem) :
    turretSubsystem(subsystem),
    manualYawPid(YAW_P, YAW_I, YAW_D, YAW_MAX_ERROR_SUM, YAW_MAX_OUTPUT),
    manualPitchPid(PITCH_P, PITCH_I, PITCH_D, PITCH_MAX_ERROR_SUM, PITCH_MAX_OUTPUT)
{
    addSubsystemRequirement(subsystem);
}

bool DroneTurretManualCommand::isFinished() const
{
    return false;
}

void DroneTurretManualCommand::end(bool) {}

void DroneTurretManualCommand::execute()
{
    updateTurretVelocity();
}

void DroneTurretManualCommand::updateTurretVelocity()
{
    pitchVelocityTarget = - USER_INPUT_SCALAR
            * aruwlib::control::ControlOperatorInterface::getTurretPitchInput();
    yawVelocityTarget = USER_INPUT_SCALAR
            * aruwlib::control::ControlOperatorInterface::getTurretYawInput();

    manualPitchPid.update(pitchVelocityTarget - turretSubsystem->getPitchVelocity());
    manualYawPid.update(yawVelocityTarget - turretSubsystem->getYawVelocity());

    turretSubsystem->setPitchMotorOutput(manualPitchPid.getValue() -
            PITCH_GRAVITY_COMPENSATION_KP * cosf(aruwlib::algorithms::degreesToRadians(
                    turretSubsystem->getPitchAngleFromCenter())));
    turretSubsystem->setYawMotorOutput(manualYawPid.getValue());
}

}  // namespace turret

}  // namespace aruwsrc
