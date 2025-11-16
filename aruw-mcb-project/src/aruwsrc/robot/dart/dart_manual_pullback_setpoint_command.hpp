#ifndef DART_MANUAL_PULLBACK_SETPOINT_COMMAND
#define DART_MANUAL_PULLBACK_SETPOINT_COMMAND

#include "tap/control/command.hpp"

#include "aruwsrc/control/joint/homing/trigger_homed_joint_subsystem.hpp"
#include "aruwsrc/robot/dart/dart_control_operator_interface.hpp"

namespace aruwsrc::robot::dart

{

class DartManualPullbackSetpointCommand : public tap::control::Command
{
public:
    DartManualPullbackSetpointCommand(
        aruwsrc::control::joint::homing::TriggerHomedJointSubsystem& dartSystem,
        float moveSpeed,
        aruwsrc::control::dart::DartControlOperatorInterface* controlOperatorInterface);
    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "DART MANUAL PULLBACK SETPOINT"; }

private:
    aruwsrc::control::joint::homing::TriggerHomedJointSubsystem& dartSystem;
    float moveSpeed;
    aruwsrc::control::dart::DartControlOperatorInterface* controlOperatorInterface;  // NOLINT
};
}  // namespace aruwsrc::robot::dart

#endif  // DART_MANUAL_PULLBACK_SETPOINT_COMMAND