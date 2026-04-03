#ifndef STICK_TORQUE_COMMAND_HPP_
#define STICK_TORQUE_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"
#include "tap/communication/serial/remote.hpp"

#include "motor_subsystem.hpp"


namespace aruwsrc::motor_tester {

class StickTorqueCommand : public tap::control::Command
{
public:
    StickTorqueCommand(
        tap::Drivers* drivers,
        tap::communication::serial::Remote::Channel channel,
        MotorSubsystem* motorSubsystem,
        float sensitivity)
        : drivers(drivers),
          channel(channel),
          motorSubsystem(motorSubsystem),
          sensitivity(sensitivity)
    {
        addSubsystemRequirement(motorSubsystem);
    }

    void execute() override
    {   
        channelValue = drivers->remote.getChannel(channel);
        motorSubsystem->setDesiredOutput(drivers->remote.getChannel(channel) * sensitivity * tap::motor::DjiMotor::MAX_OUTPUT_C620);
    }

    void initialize() override {}
    void end(bool) override {motorSubsystem->setDesiredOutput(0); }
    bool isFinished() const override { return false; }
    const char* getName() const override { return "StickTorqueCommand"; }

private:
    tap::Drivers* drivers;
    tap::communication::serial::Remote::Channel channel;
    MotorSubsystem* motorSubsystem;
    float channelValue;
    float sensitivity;
};
}


#endif // STICK_TORQUE_COMMAND_HPP_