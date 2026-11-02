
#ifndef MOTOR_TESTER_COMMAND_HPP_
#define MOTOR_TESTER_COMMAND_HPP_

#include <algorithm>
#include <cstdint>

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"
#include "motor_tester_subsystem.hpp"

namespace aruwsrc::motor_tester
{

class MotorTesterCommand : public tap::control::Command
{
public:
    MotorTesterCommand(
    tap::Drivers* drivers,
    MotorTesterSubsystem& motorSubsystem,
    tap::communication::serial::Remote::Channel channel,
    float sensitivity)
    : drivers(drivers),
      motorSubsystem(motorSubsystem),
      channel(channel),
      sensitivity(sensitivity),
      desiredPosition(0.0f)  
{
    addSubsystemRequirement(&motorSubsystem);
}

    void initialize() override
{
    desiredPosition = motorSubsystem.getDesiredPosition();
}

    void execute() override
{
    if (!drivers->remote.isConnected())
    {
        return;
    }

    float input = drivers->remote.getChannel(channel);

    desiredPosition += input * sensitivity * 0.002f;

    motorSubsystem.setDesiredPosition(desiredPosition);
}

    void end(bool interrupted) override
    {
        motorSubsystem.setDesiredPosition(0);
    }

    bool isFinished() const override
    {
        return false;
    }

    const char* getName() const override
    {
        return "Motor Tester Command";
    }

private:
    tap::Drivers* drivers;
    MotorTesterSubsystem& motorSubsystem;
    tap::communication::serial::Remote::Channel channel;
    float sensitivity;
    float desiredPosition;
};

}  // namespace aruwsrc::motor_tester

#endif  // MOTOR_TESTER_COMMAND_HPP_
