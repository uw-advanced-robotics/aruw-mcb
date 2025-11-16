#include "dart_manual_pullback_setpoint_command.hpp"

namespace aruwsrc::robot::dart

{

DartManualPullbackSetpointCommand::DartManualPullbackSetpointCommand(
    aruwsrc::control::joint::homing::TriggerHomedJointSubsystem& dartSystem,
    float moveSpeed,
    aruwsrc::control::dart::DartControlOperatorInterface* controlOperatorInterface)  // NOLINT
    : dartSystem(dartSystem),
      moveSpeed(moveSpeed),
      controlOperatorInterface(controlOperatorInterface)
{
    addSubsystemRequirement(&dartSystem);
}

void DartManualPullbackSetpointCommand::initialize() {}

void DartManualPullbackSetpointCommand::execute()
{
    float setpoint = dartSystem.getSetpoint();

    setpoint += controlOperatorInterface->getPullbackVelocity() * moveSpeed;

    dartSystem.setSetpoint(setpoint);
}

bool DartManualPullbackSetpointCommand::isFinished() const { return false; }

void DartManualPullbackSetpointCommand::end(bool interrupted) {}
}  // namespace aruwsrc::robot::dart