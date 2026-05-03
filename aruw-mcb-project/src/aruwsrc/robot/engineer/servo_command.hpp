#ifndef SERVO_COMMAND_HPP_
#define SERVO_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "servo_subsystem.hpp"

namespace aruwsrc::engineer
{
class ServoCommand : public tap::control::Command
{
public:
    ServoCommand(ServoSubsystem& subsystem) : subsystem(subsystem)
    {
        addSubsystemRequirement(&subsystem);
    }

    const char* getName() const override { return "Servo Command"; }

    bool isReady() override { return true; }
    
    void initialize() override {
        subsystem.servoOne.setTargetPwm(0.5f);
        subsystem.servoTwo.setTargetPwm(0.5f);
    }
    
    void execute() override {}

    void end(bool interrupted) override {}

    bool isFinished() const override { return false; }

private:
    ServoSubsystem& subsystem;
};
}
#endif