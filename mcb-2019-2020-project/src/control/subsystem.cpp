#include "src/control/subsystem.hpp"
#include "src/control/command.hpp"
#include "scheduler.hpp"

namespace aruwlib
{

namespace control
{
    Subsystem::Subsystem()
    {
        CommandScheduler::registerSubsystem1(this);
    }

    void Subsystem::SetDefaultCommand(Command* command)
    {
        defaultCommand = command;
    }

    Command* Subsystem::GetDefaultCommand() const
    {
        return defaultCommand;
    }
}  // namespace control

}  // namespace aruwlib
