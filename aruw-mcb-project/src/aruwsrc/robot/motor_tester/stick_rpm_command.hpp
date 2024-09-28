
#ifndef STICK_RPM_COMMAND_HPP_
#define STICK_RPM_COMMAND_HPP_

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "motor_subsystem.hpp"

using namespace aruwsrc::motor_tester;

class StickRpmCommand : public tap::control::Command
{
public:
    explicit StickRpmCommand(
        MotorSubsystem* subsystem,
        tap::communication::serial::Remote* remote,
        tap::communication::serial::Remote::Channel channel,
        float maxRpm);

    void initialize() override {}

    void execute() override;

    void end(bool) override;

    bool isFinished() const override { return false; }

    const char* getName() const override { return "stick rpm"; }

private:
    MotorSubsystem* motorSubsystem;
    tap::communication::serial::Remote* remote;
    tap::communication::serial::Remote::Channel channel;
    float maxRpm;

    const float STICK_DEADZONE = 0.05;
};  // class StickRpmCommand

#endif  // STICK_RPM_COMMAND_HPP_
