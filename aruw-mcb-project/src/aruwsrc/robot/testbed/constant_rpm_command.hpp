
#ifndef CONSTANT_RPM_COMMAND_HPP_
#define CONSTANT_RPM_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "motor_subsystem.hpp"

class ConstantRpmCommand : public tap::control::Command
{
public:
    explicit ConstantRpmCommand(MotorSubsystem* subsystem, float rpm, float pulleyRatio = 1.0f);

    void initialize() override {}

    void execute() override;

    void end(bool) override;

    bool isFinished() const override { return false; }

    const char* getName() const override { return "constant rpm"; }

private:
    MotorSubsystem* motorSubsystem;
    float rpm, pulleyRatio;
};  // class ConstantRpmCommand

#endif  // CONSTANT_RPM_COMMAND_HPP_
