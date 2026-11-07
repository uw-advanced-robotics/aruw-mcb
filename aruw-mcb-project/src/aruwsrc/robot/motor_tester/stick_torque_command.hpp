#ifndef STICK_TORQUE_COMMAND_HPP_
#define STICK_TORQUE_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/communication/serial/remote.hpp"
#include "motor_subsystem.hpp"

namespace aruwsrc::motor_tester
{
class StickTorqueCommand : public tap::control::Command
{
public:
    StickTorqueCommand(
        tap::Drivers* drivers,
        tap::communication::serial::Remote* remote,
        tap::communication::serial::Remote::Channel channel,
        motor_tester::MotorSubsystem* motor,
        float scaleFactor
    );

    bool isReady() override;
    const char* getName() const override {return "Motor tester";}
    void initialize() override;
    void execute() override;
    bool isFinished() const override;
    void end(bool) override;

private:
    tap::Drivers* drivers;
    tap::communication::serial::Remote* remote;
    tap::communication::serial::Remote::Channel channel;
    motor_tester::MotorSubsystem* motor;
    float scaleFactor;
};
}

#endif