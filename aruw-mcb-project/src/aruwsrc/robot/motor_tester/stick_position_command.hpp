#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"

#include "motor_subsystem.hpp"

#ifndef STICK_POSITION_COMMAND
#define STICK_POSITION_COMMAND
namespace aruwsrc::robot::motor_tester
{
class StickPositionCommand : public tap::control::Command
{
public:
    StickPositionCommand(
        tap::Drivers* drivers,
        tap::communication::serial::Remote::Channel channel,
        MotorSubsystem* motorSubsystem,
        float scalar)
        : drivers(drivers),
          channel(channel),
          motorSubsystem(motorSubsystem),
          scalar(scalar)
    {
        addSubsystemRequirement(motorSubsystem);
    }
    void initialize() override {}

    void execute() override
    {
        motorSubsystem->setDesiredPosition(drivers->remote.getChannel(channel) * scalar);
    }

    void end(bool) override { motorSubsystem->setDesiredOutput(0); }

    bool isFinished() const override { return false; }

    const char* getName() const override { return "a"; }

private:
    tap::Drivers* drivers;
    tap::communication::serial::Remote::Channel channel;
    MotorSubsystem* motorSubsystem;
    float scalar;
};

}  // namespace aruwsrc::robot::motor_tester
#endif  // STICK_POSITION_COMMAND