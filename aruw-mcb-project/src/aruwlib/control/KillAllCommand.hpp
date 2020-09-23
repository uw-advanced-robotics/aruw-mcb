#ifndef KILL_ALL_COMMAND_HPP_
#define KILL_ALL_COMMAND_HPP_

#include <aruwlib/control/command.hpp>
#include <modm/processing/timer/timeout.hpp>

#include "KillAllSubsystem.hpp"

using namespace aruwlib::control;

namespace aruwlib
{
namespace control
{
class KillAllCommand : public aruwlib::control::Command
{
public:
    KillAllCommand(KillAllSubsystem *killAllSubsystem, aruwlib::Drivers *drivers);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char *getName() const { return "kill all command"; }

private:
    KillAllSubsystem *killAllSubsystem;

    aruwlib::Drivers *drivers;
};  // class KillAllCommand
}  // namespace control
}  // namespace aruwlib

#endif  // KILL_ALL_COMMAND_HPP_
