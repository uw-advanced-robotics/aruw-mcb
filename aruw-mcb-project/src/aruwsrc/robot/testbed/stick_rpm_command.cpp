#include "stick_rpm_command.hpp"

StickRpmCommand::StickRpmCommand(
    MotorSubsystem* subsystem,
    tap::communication::serial::Remote* remote,
    tap::communication::serial::Remote::Channel channel,
    float maxRpm)
    : motorSubsystem(subsystem),
      remote(remote),
      channel(channel),
      maxRpm(maxRpm)
{
    this->addSubsystemRequirement(subsystem);
}

void StickRpmCommand::execute()
{
    float stick = remote->getChannel(this->channel);

    // math for this (unused atm but still): https://www.desmos.com/calculator/ip8m03ugmo
    float deadenedStick = 0;
    if (stick > STICK_DEADZONE)
    {
        deadenedStick = (stick - STICK_DEADZONE) / (1 - STICK_DEADZONE);
    }
    else if (stick < -STICK_DEADZONE)
    {
        deadenedStick = (stick + STICK_DEADZONE) / (1 - STICK_DEADZONE);
    }
    motorSubsystem->setDesiredRPM(maxRpm * stick);
    motorSubsystem->refresh();
}

void StickRpmCommand::end(bool) { motorSubsystem->stop(); }