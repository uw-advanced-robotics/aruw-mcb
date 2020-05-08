#include "drone_turret_init_command.hpp"
#include "drone_turret_subsystem.hpp"
#include <aruwlib/control/command.hpp>
#include <aruwlib/algorithms/math_user_utils.hpp>

namespace aruwsrc
{

namespace turret
{

DroneTurretInitCommand::DroneTurretInitCommand(DroneTurretSubsystem *subsystem) :
    turretSubsystem(subsystem),
    initYawPid(YAW_P, YAW_I, YAW_D, YAW_MAX_ERROR_SUM, YAW_MAX_OUTPUT),
    initPitchPid(PITCH_P, PITCH_I, PITCH_D, PITCH_MAX_ERROR_SUM, PITCH_MAX_OUTPUT)
{
    addSubsystemRequirement(subsystem);
}

bool DroneTurretInitCommand::isFinished() const
{
    return fabsf(turretSubsystem->getPitchAngleFromCenter()) < 3.0f
        && fabsf(turretSubsystem->getYawAngleFromCenter()) < 3.0f
        && turretSubsystem->isTurretOnline();
}

void DroneTurretInitCommand::end(bool) {}

void DroneTurretInitCommand::execute()
{
    updateTurretPosition();
}

void DroneTurretInitCommand::updateTurretPosition()
{
    initPitchPid.update(turretSubsystem->getPitchAngle().difference(pitchTargetAngle));
    initYawPid.update(turretSubsystem->getYawAngle().difference(yawTargetAngle));
    turretSubsystem->setPitchMotorOutput(initPitchPid.getValue() -
            PITCH_GRAVITY_COMPENSATION_KP * cosf(aruwlib::algorithms::degreesToRadians(
                    turretSubsystem->getPitchAngleFromCenter())));
    turretSubsystem->setYawMotorOutput(initYawPid.getValue());
}

}  // namespace turret

}  // namespace aruwsrc
