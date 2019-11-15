#include "src/control/subsystem.hpp"
#include "src/control/command.hpp"
#include "scheduler.hpp"

namespace aruwlib
{

namespace control
{
    Subsystem::Subsystem()
    {
       CommandScheduler::registerSubsystem(this);
    }

    void Subsystem::SetDefaultCommand(Command* command)
    {
        defaultCommand = command;
    }

    Command* Subsystem::GetDefaultCommand() const
    {
        return defaultCommand;
    }

    void Subsystem::SetCurrentCommand(Command* command)
    {
        currentCommand = command;
    }

    Command* Subsystem::GetCurrentCommand() const
    {
        return currentCommand;
    }
}  // namespace control

}  // namespace aruwlib
