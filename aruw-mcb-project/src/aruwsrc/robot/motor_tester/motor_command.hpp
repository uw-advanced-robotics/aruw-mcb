#include "tap/control/command.hpp"
#include "aruwsrc/robot/motor_tester/motor_subsystem.hpp"
#include "tap/control/command_mapping/analog_remote_mapping.hpp"


class MotorCommand : public tap::control::Command
{
public:
    inline MotorCommand(MotorSubsystem& motorSubsystem, tap::control::AnalogRemoteMapping& analogInput) : motorSubsystem(motorSubsystem), analogInput(analogInput)
    {
        this->addSubsystemRequirement(&motorSubsystem);
    }
    inline const char* getName() const override final { return "motorCommand"; }
    inline void initialize() override final {}
    inline void execute() override final
    {
        this->motorSubsystem.setMotorOutput(analogInput.getValue());
    }
    inline void end(bool isInterrupted) override final
    {
        this->motorSubsystem.setMotorOutput(0);
    }
    inline bool isFinished() const override final { return false; }
private:
    MotorSubsystem& motorSubsystem;
    tap::control::AnalogRemoteMapping& analogInput;
};